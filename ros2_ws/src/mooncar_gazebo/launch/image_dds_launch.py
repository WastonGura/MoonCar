"""Launch adapter; environment overrides are confined to the bridge child."""
import importlib.util, json, os, sys, tempfile
from pathlib import Path
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, LogInfo
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def launch_bridge(context):
    spec=importlib.util.spec_from_file_location('image_dds_guard',Path(__file__).with_name('image_dds_guard.py'))
    guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)
    package=Path(FindPackageShare('mooncar_gazebo').perform(context))
    default=guard.default_profile(package/'config/image_dds_defaults.json')
    support=LaunchConfiguration('simulation_profile',default='legacy').perform(context)=='r5_support_v1'
    choice=guard.select(LaunchConfiguration('model_file',default='mooncar.urdf.xacro').perform(context),LaunchConfiguration('world').perform(context),os.environ,default,support_candidate=support)
    config=package/'config'/LaunchConfiguration('bridge_config_file',default='ros_gz_bridge.yaml').perform(context)
    if choice['selected']=='legacy':
        return [LogInfo(msg='MoonCar image DDS: '+json.dumps(choice)),IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('ros_gz_bridge'),'launch','ros_gz_bridge.launch.py'])),
            launch_arguments={'bridge_name':'mooncar_gz_bridge','config_file':str(config)}.items())]
    root=os.environ.get('MOONCAR_IMAGE_DDS_RUNTIME','/opt/mooncar/image-dds/rmw-8.4.4');guard.verify(root)
    from ament_index_python.packages import get_package_prefix
    binary=Path(get_package_prefix('ros_gz_bridge'))/'lib/ros_gz_bridge/bridge_node'
    parent=Path(os.environ.get('MOONCAR_IMAGE_DDS_RECEIPT_ROOT',tempfile.gettempdir()))
    parent.mkdir(parents=True,exist_ok=True)
    folder=parent/('mooncar-image-dds-'+next(tempfile._get_candidate_names()))
    bridge=ExecuteProcess(cmd=[sys.executable,str(Path(__file__).with_name('image_dds_guard.py')),
        '--root',root,'--receipt-directory',str(folder),'--binary',str(binary),
        '--ros-args','-r','__node:=mooncar_gz_bridge','-p','config_file:='+str(config)],output='screen')
    return [LogInfo(msg='MoonCar image DDS: '+json.dumps(dict(**choice,receipt_directory=str(folder)))),
        RegisterEventHandler(OnProcessExit(target_action=bridge,on_exit=[EmitEvent(event=Shutdown(reason='Image DDS bridge exited'))])),bridge]
