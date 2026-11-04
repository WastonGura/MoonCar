"""Select the scoped runtime before world path rewriting; no global LD override."""
import json
import os
import tempfile
from pathlib import Path
from launch.actions import ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler, EmitEvent, SetLaunchConfiguration
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import importlib.util

def guard():
    spec=importlib.util.spec_from_file_location('turn_guard',Path(__file__).with_name('turn_guard.py'))
    value=importlib.util.module_from_spec(spec);spec.loader.exec_module(value);return value

def configure(context):
    support=LaunchConfiguration('simulation_profile',default='legacy').perform(context)
    if support not in ('legacy','r5_support_v1'):raise RuntimeError('Invalid simulation_profile')
    if support=='r5_support_v1':
        if LaunchConfiguration('model_file').perform(context)!='mooncar_r5.urdf.xacro':
            raise RuntimeError('R5 support candidate requires the R5 model')
        if os.environ.get('MOONCAR_TURN_PROFILE','legacy')!='legacy':
            raise RuntimeError('Support candidate cannot be combined with adaptive steering')
        spec=importlib.util.spec_from_file_location('support_profile',Path(__file__).with_name('support_profile.py'))
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        root=os.environ.get('MOONCAR_SUPPORT_RUNTIME','/opt/mooncar/turn/r5-v2')
        module.verify(root)
        return [SetLaunchConfiguration('_turn_selected',support),SetLaunchConfiguration('_turn_root',root),
                LogInfo(msg='MoonCar simulation: r5_support_v1; independent_linear_v1; r5_precision_v1 (0.25 ms); fixed allocation; uncalibrated')]
    package=Path(FindPackageShare('mooncar_gazebo').perform(context))
    default=json.loads((package/'config/turn_defaults.json').read_text())
    if set(default)!={'profile'} or default['profile'] not in ('auto','legacy'):
        raise RuntimeError('Invalid steering default configuration')
    requested=os.environ.get('MOONCAR_TURN_PROFILE',default['profile'])
    selected=guard().select(LaunchConfiguration('model_file').perform(context),LaunchConfiguration('world').perform(context),requested)
    release={'approved':False,'reason':'Repair not selected'}
    if selected=='r5_v2':
        release_path=package/'config/turn_release.json'
        release=json.loads(release_path.read_text()) if release_path.is_file() else {'approved':False,'reason':'Release approval missing'}
    if selected=='r5_v2' and release.get('approved') is not True:
        if requested=='r5_v2':raise RuntimeError('Steering repair has not passed release: '+release.get('reason','Missing approval'))
        selected='legacy'
    root=os.environ.get('MOONCAR_TURN_RUNTIME','/opt/mooncar/turn/r5-v2')
    if selected=='r5_v2':guard().verify(root)
    return [SetLaunchConfiguration('_turn_selected',selected),SetLaunchConfiguration('_turn_root',root),
            LogInfo(msg='MoonCar turn: '+json.dumps(dict(requested=requested,selected=selected,root=root,release=release)))]

def simulator(context):
    selected=LaunchConfiguration('_turn_selected').perform(context)
    if selected=='legacy':
        return [IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_sim'),'launch','gz_sim.launch.py'])),
            launch_arguments={'gz_args':[LaunchConfiguration('gz_args'),' ',LaunchConfiguration('world')],'on_exit_shutdown':'true'}.items())]
    import sys
    process=ExecuteProcess(cmd=[sys.executable,str(Path(__file__).with_name('turn_runtime.py')),
        '--root',LaunchConfiguration('_turn_root'), '--gz-args',LaunchConfiguration('gz_args'),
        '--world',LaunchConfiguration('world'),'--profile',selected],output='screen')
    if selected=='r5_support_v1':
        receipt_root=Path(os.environ.get('MOONCAR_SUPPORT_RECEIPT_ROOT',tempfile.mkdtemp(prefix='mooncar-support-receipt-')))
        receipt_root.mkdir(parents=True,exist_ok=True)
        process=ExecuteProcess(cmd=[sys.executable,str(Path(__file__).with_name('turn_runtime.py')),
            '--root',LaunchConfiguration('_turn_root'),'--gz-args',LaunchConfiguration('gz_args'),
            '--world',LaunchConfiguration('world'),'--profile',selected,'--receipt',str(receipt_root/'loaded.json')],output='screen')
        return [RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='Support simulator exited'))])),process]
    feedback=Node(package='mooncar_control',executable='r5_adaptive_turn_node.py',
        arguments=['--output',tempfile.mkdtemp(prefix='mooncar-adaptive-turn-')],output='screen')
    bridge=Node(package='ros_gz_bridge',executable='parameter_bridge',
        arguments=['/mooncar/turn_allocation@std_msgs/msg/String]gz.msgs.StringMsg'],output='screen')
    return [RegisterEventHandler(OnProcessExit(target_action=process,on_exit=[EmitEvent(event=Shutdown(reason='Steering simulator exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=feedback,on_exit=[EmitEvent(event=Shutdown(reason='Adaptive feedback exited'))])),
        RegisterEventHandler(OnProcessExit(target_action=bridge,on_exit=[EmitEvent(event=Shutdown(reason='Adaptive bridge exited'))])),
        bridge,feedback,process]
