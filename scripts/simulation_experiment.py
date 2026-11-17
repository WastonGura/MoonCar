"""Own an isolated Gazebo experiment, camera view and explicitly leased Nav2 goals."""
import json
import copy
import math
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET
import numpy as np
from survey_evaluation import write_json
from experiment_scenes import (SCENES, V2_SCENES, V2_WORLD, V3_SCENES, V3_WORLD, CONTINUOUS_SCENES, add_v2_localization_markers,
                               scene_bounds, scene_metadata, validate_v2_spawn,
                               v2_marker_metadata, v2_spawn_obstacles,
                               verify_installed_scene_resources)

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / 'ros2_ws/src/mooncar_gazebo/worlds/lunar_yard.sdf'


def experiment_domain():
    value = os.environ.get('MOONCAR_EXPERIMENT_ROS_DOMAIN_ID', '89')
    if not value.isdecimal() or not 0 <= int(value) <= 232:
        raise ValueError('MOONCAR_EXPERIMENT_ROS_DOMAIN_ID must be an integer in 0..232')
    return str(int(value))


def transform(element):
    pose = element.find('pose')
    if pose is not None and any(k != 'relative_to' for k in pose.attrib):
        raise ValueError('Unsupported world pose reference')
    x, y, z, r, p, a = map(float, element.findtext('pose', '0 0 0 0 0 0').split())
    cr, sr, cp, sp, ca, sa = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(a), math.sin(a)
    matrix = np.eye(4)
    matrix[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                      [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr], [-sp, cp*sr, cp*cr]]
    matrix[:3, 3] = [x, y, z]
    return matrix


def survey_world(scene='survey'):
    """Selected physical world; never used as an observation map."""
    if scene not in SCENES:
        raise ValueError('Unknown experiment scene')
    if scene == 'survey_v2':
        # Preserve all shipped V2 terrain/physics; the visible fixtures provide
        # real laser returns above the unchanged legacy lidar's mounting height.
        return add_v2_localization_markers(ET.parse(V2_WORLD))
    if scene == 'survey_v2_native':
        return ET.parse(V2_WORLD)
    if scene in V3_SCENES:return ET.parse(V3_WORLD)
    tree=ET.parse(WORLD);world=tree.getroot().find('world')
    if scene!='survey':
        # Controlled visibility fixtures: retain the same physics, sensors and ground.
        ramp=copy.deepcopy(world.find("model[@name='gentle_ramp']"))
        rock=copy.deepcopy(world.find("model[@name='scattered_rock_c']"))
        for model in list(world.findall('model')):
            if model.get('name')!='lunar_regolith_plane':world.remove(model)
        ramp.set('name','visible_target')
        ramp.find('pose').text='3 0 0.055 0 -0.08 0' if scene=='visible_ramp_v1' else '3 0 -0.08 0 0 0.25'
        for geometry in ramp.findall('.//geometry'):
            geometry.clear()
            shape=ET.SubElement(geometry,'box' if scene=='visible_ramp_v1' else 'ellipsoid')
            ET.SubElement(shape,'size' if scene=='visible_ramp_v1' else 'radii').text='2 2.4 0.08' if scene=='visible_ramp_v1' else '1.1 1.4 0.25'
        world.append(ramp)
        # Distant lidar landmarks support localization without hiding the target.
        for i,(x,y) in enumerate(((-5,-4),(-4,5),(6,-4),(7,5))):
            marker=copy.deepcopy(rock);marker.set('name',f'localization_marker_{i}')
            pose=marker.find('pose');values=pose.text.split();values[:2]=map(str,(x,y));pose.text=' '.join(values)
            world.append(marker)
        return tree
    templates=[world.find(f"model[@name='{name}']") for name in
               ('scattered_rock_c','gentle_ramp','north_ridge','rock_field_marker_a')]
    count=0
    for row,y in enumerate((-20,-12,-4,4,12,20)):
        for col,x in enumerate((-20,-12,-4,4,12,20)):
            if abs(x)<8 and abs(y)<8:continue
            model=copy.deepcopy(templates[(row+col)%len(templates)])
            model.set('name',f'survey_landmark_{count}');count+=1
            pose=model.find('pose');values=list(map(float,pose.text.split()))
            values[0]=x;values[1]=y;values[5]=(row%3)*.4
            pose.text=' '.join(map(str,values));world.append(model)
    return tree


def spawn_obstacles(scene='survey'):
    # Conservative bounds are setup-only world metadata, never sensor observations.
    if scene in CONTINUOUS_SCENES:
        return v2_spawn_obstacles(include_markers=scene == 'survey_v2',scene=scene)
    obstacles = []
    for model in survey_world(scene).findall('./world/model'):
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                g = collision.find('geometry')
                if g.find('plane') is not None:
                    continue
                if g.find('box') is not None:
                    extent = np.array(list(map(float, g.findtext('box/size').split()))) / 2
                elif g.find('ellipsoid') is not None:
                    extent = np.array(list(map(float, g.findtext('ellipsoid/radii').split())))
                elif g.find('cylinder') is not None:
                    extent = np.array([float(g.findtext('cylinder/radius'))]*2 + [float(g.findtext('cylinder/length'))/2])
                else:
                    raise ValueError('Unsupported spawn collision geometry')
                t = transform(model) @ transform(link) @ transform(collision)
                half = np.abs(t[:3,:3]) @ extent
                obstacles.append((t[:2,3]-half[:2], t[:2,3]+half[:2]))
    return obstacles


def validate_task(body):
    if not isinstance(body, dict) or set(body)-{'scene','gui','simulation_profile'} != {'version', 'region', 'start'} or type(body['version']) is not int or body['version'] != 1:
        raise ValueError('任务格式无效，请重新保存任务')
    scene=body.get('scene','survey')
    profile=body.get('simulation_profile','legacy')
    if profile not in ('legacy','r5_support_v1'):raise ValueError('不支持的仿真模型配置')
    if not isinstance(scene,str) or scene not in SCENES:
        raise ValueError('不支持的实验场景')
    if type(body.get('gui',True)) is not bool:raise ValueError('GUI 配置必须为布尔值')
    region, start = body['region'], body['start']
    if not isinstance(region, list) or len(region) != 4 or not isinstance(start, list) or len(start) != 3:
        raise ValueError('需要矩形区域及起点 X、Y、朝向')
    if not all(type(v) in (int, float) and math.isfinite(v) for v in region+start):
        raise ValueError('坐标和朝向必须为有限数值')
    x0, y0, x1, y1 = region
    x, y, yaw = start
    low_x, low_y, high_x, high_y = scene_bounds(scene)
    if (not (low_x <= x0 <= high_x and low_x <= x1 <= high_x and
             low_y <= y0 <= high_y and low_y <= y1 <= high_y and
             low_x <= x <= high_x and low_y <= y <= high_y)
            or not (.5 <= x1-x0 <= high_x-low_x and .5 <= y1-y0 <= high_y-low_y)
            or abs(yaw) > math.pi):
        raise ValueError(f'坐标范围为 {low_x:g} 至 {high_x:g} 米；区域每边至少 0.5 米；朝向为 -180 至 180 度')
    hx = .43*abs(math.cos(yaw)) + .36*abs(math.sin(yaw)) + .1
    hy = .43*abs(math.sin(yaw)) + .36*abs(math.cos(yaw)) + .1
    for low, high in spawn_obstacles(scene):
        if x+hx >= low[0] and x-hx <= high[0] and y+hy >= low[1] and y-hy <= high[1]:
            raise ValueError('起点车体与场景障碍或坡面重叠，请选择周围平地')
    if scene in CONTINUOUS_SCENES:
        validate_v2_spawn(x, y, yaw, include_markers=scene == 'survey_v2',scene=scene)
    return dict(version=1, region=list(map(float, region)), start=list(map(float, start)),
                **({'scene':scene} if 'scene' in body else {}),**({'gui':body['gui']} if 'gui' in body else {}),
                **({'simulation_profile':profile} if 'simulation_profile' in body else {}))


class SimulationExperiment:
    def __init__(self, directory):
        if os.name != 'posix' or not os.environ.get('ROS_DISTRO'):
            raise ValueError('仿真模式需要已 source ROS 的独立 Ubuntu 环境')
        from ament_index_python.packages import get_package_share_directory
        verify_installed_scene_resources(get_package_share_directory('mooncar_gazebo'))
        self.directory = Path(directory).resolve()
        from runtime_baseline import load_identity
        self.software=load_identity(os.environ.get('MOONCAR_RUNTIME_MANIFEST'))
        self.lock = threading.Lock()
        self.process = self.log = None
        self.current = None
        self.phase, self.error = 'idle', ''
        self.worker = None
        self.started_at = None
        self.startup_failure = None
        self.tf_refreshed = False
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.signals import SignalHandlerOptions
        from experiment_motion import ExperimentMotion
        laser_setting=os.environ.get('MOONCAR_LASER_ODOMETRY','true').lower()
        if laser_setting not in ('true','false'):
            raise ValueError('MOONCAR_LASER_ODOMETRY must be true or false')
        laser_mode=os.environ.get('MOONCAR_LASER_ODOMETRY_MODE','rgbd_window').lower()
        if laser_mode not in ('ekf','direct','terrain','rgbd','rgbd_window'):
            raise ValueError('MOONCAR_LASER_ODOMETRY_MODE must be ekf, direct, terrain, rgbd or rgbd_window')
        self.laser_odometry=laser_setting=='true'
        self.laser_odometry_mode=laser_mode
        self.ros_domain = experiment_domain()
        os.environ['ROS_DOMAIN_ID']=self.ros_domain
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],signal_handler_options=SignalHandlerOptions.NO)
        self.motion=ExperimentMotion(require_laser_localization=self.laser_odometry,
            gimbal_command_rate_rad_s=.35 if self.laser_odometry and self.laser_odometry_mode in ('rgbd','rgbd_window') else None)
        from autonomous_survey import AutonomousSurvey
        self.autonomy=AutonomousSurvey(self.motion,
            initial_depth_scan=self.laser_odometry and self.laser_odometry_mode in ('terrain','rgbd','rgbd_window'),
            localization_revisit_enabled=self.laser_odometry_mode=='rgbd',
            wall_limit_seconds=float(os.environ.get('MOONCAR_AUTONOMY_WALL_LIMIT_SECONDS','3600')))
        self.executor=MultiThreadedExecutor(num_threads=4);self.executor.add_node(self.motion)
        self.ros_thread=threading.Thread(target=self.executor.spin,daemon=True);self.ros_thread.start()

    def state(self):
        with self.lock:
            current = self.current
            phase, error = self.phase, self.error
            if phase == 'starting' and self.process and self.process.poll() is not None:
                phase, error = 'failed', '仿真进程退出，请检查该实验的 simulation.log'
            archives = sorted((self.directory/current['id']).glob('*.npz')) if current else []
            # "recording" means a new sensor archive exists, not that area exploration ran.
            if phase == 'starting' and archives:
                phase = 'recording'
            elif phase == 'starting' and self.started_at and time.monotonic()-self.started_at > 120:
                phase, error = 'failed', '超过 120 秒未收到采集档案，请检查定位、传感器和 simulation.log；可结束后重试'
            motion=self.motion.state() if hasattr(self,'motion') else {}
            if (phase in ('starting','recording') and current and self.started_at
                    and time.monotonic()-self.started_at > 10
                    and not getattr(self,'tf_refreshed',False) and motion.get('pose') is None):
                self.motion.refresh_tf_listener()
                self.tf_refreshed = True
                motion = self.motion.state()
            if current and current.get('start') and motion.get('pose') and not self.motion.start_checked:
                pose=motion['pose'];start=current['start']
                distance=math.hypot(pose['x']-start[0],pose['y']-start[1])
                motion.setdefault('diagnostics',{})['planned_start_error_m']=round(distance,3)
                motion['ready']=bool(motion.get('ready')) and distance<=.25
            if phase=='failed' and current and getattr(self,'startup_failure',None) is None:
                self.startup_failure=dict(reason=error,experiment=current['id'],
                    elapsed_wall_seconds=time.monotonic()-self.started_at if self.started_at else None,
                    launch_exit_code=self.process.poll() if self.process else None,
                    archive_count=len(archives),health=motion.get('diagnostics'),
                    motion_phase=motion.get('phase'),motion_error=motion.get('error'))
                try:write_json(self.directory/current['id']/'startup-failure.json',self.startup_failure)
                except OSError as exc:self.startup_failure['write_error']=str(exc)
            return dict(enabled=True, phase=phase, error=error, task=current,
                        software=getattr(self,'software',dict(verified=False)),
                        motion=motion,startup_failure=getattr(self,'startup_failure',None),
                        autonomy=self.autonomy.state() if hasattr(self,'autonomy') else {},
                        session=str(archives[-1].relative_to(self.directory)).replace('\\','/') if archives else None)

    def inspect_cell(self, session, x, y):
        from map_cell_inspection import inspect_cell
        with self.lock:
            if (self.phase!='starting' or not self.current or not session.startswith(self.current['id']+'/')
                    or not self.process or self.process.poll() is not None):
                return dict(status='unavailable')
            path = self.directory/self.current['id']/'terrain-cell-evidence.json'
            with self.motion.lock:
                grid = self.motion.grid_view if self.motion.grid_fresh() else None
                keepout = self.motion.keepout_grid
        try:
            evidence = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, ValueError):
            evidence = None
        return inspect_cell(grid, evidence, keepout, x, y)

    def image(self):
        if not self.current or self.phase not in ('starting',):
            raise ValueError('仿真现场尚未启动')
        path=self.directory/self.current['id']/'scene.jpg'
        if not path.is_file() or time.time()-path.stat().st_mtime>3:
            raise ValueError('等待 Gazebo 现场画面')
        return path.read_bytes()

    def command(self,action,body):
        with self.lock:
            if action=='cancel':
                self.autonomy.stop()
                return self.motion.cancel('已手动停止导航')
            if action=='explore_stop':return self.autonomy.stop()
            if action=='heartbeat':return self.motion.heartbeat(body)
            if action=='explore' and self.phase=='starting' and self.current and self.process.poll() is None:
                return self.autonomy.start(body,self.current,self.directory)
            if action=='pan_tilt' and self.phase=='starting' and self.current and self.process.poll() is None:
                if self.autonomy.state()['active']:raise ValueError('请先结束自主探测，再指定云台观察')
                if (not isinstance(body,dict) or set(body)!={'experiment','pan','tilt'}
                        or body['experiment']!=self.current['id']):
                    raise ValueError('实验或云台观察请求无效')
                return self.motion.pan_tilt(body['pan'],body['tilt'],self.current)
            if action=='turn' and self.phase=='starting' and self.current and self.process.poll() is None:
                if self.autonomy.state()['active']:raise ValueError('请先结束自主探测，再指定观察朝向')
                if not isinstance(body,dict) or set(body)!={'experiment','yaw'} or body['experiment']!=self.current['id']:
                    raise ValueError('实验或观察朝向请求无效')
                return self.motion.turn(body['yaw'],self.current)
            if action=='move' and self.phase=='starting' and self.current and self.process.poll() is None:
                if self.autonomy.state()['active']:raise ValueError('请先结束自主探测，再手动指定目标')
                return self.motion.start(body,self.current)
            raise ValueError('当前仿真实验不可执行导航')

    def _stop(self):
        if hasattr(self,'autonomy'):
            self.autonomy.stop('实验正在结束')
            if self.autonomy.worker:self.autonomy.worker.join(timeout=10)
        if hasattr(self,'motion'):self.motion.cancel('实验正在结束')
        def alive_group(pgid):
            # Container PID 1 may retain exited grandchildren as zombies; they cannot run.
            for entry in Path('/proc').glob('[0-9]*/stat'):
                try:
                    fields = entry.read_text().rsplit(')',1)[1].split()
                    if int(fields[2]) == pgid and fields[0] not in ('Z','X'):
                        return True
                except (OSError, ValueError, IndexError):
                    continue
            return False
        if self.process is not None:
            pgid = self.process.pid
            for sig, timeout in ((signal.SIGINT, 12), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic()+timeout
                while time.monotonic() < deadline:
                    self.process.poll()
                    if not alive_group(pgid):
                        break
                    time.sleep(.1)
                else:
                    continue
                break
            else:
                raise RuntimeError('旧仿真进程未退出，禁止创建新实验')
            self.process.wait(timeout=2)
            self.process = None
        # Old publishers must be gone before clearing their cached TF/map/quality.
        if hasattr(self,'motion'):self.motion.reset()
        if self.log:
            self.log.close()
            self.log = None

    def apply(self, body):
        task = validate_task(body)  # Reject bad requests before stopping the current experiment.
        scene = task.get('scene', 'survey')
        setup = scene_metadata(scene, self.laser_odometry_mode if self.laser_odometry else None)
        setup.update(validate_v2_spawn(*task['start'], include_markers=scene == 'survey_v2',scene=scene)
                     if scene in CONTINUOUS_SCENES else dict(initial_z=0., setup_only=True))
        setup['localization_markers'] = list(v2_marker_metadata()) if scene == 'survey_v2' else []
        setup['render_engine'] = 'ogre2' if scene in CONTINUOUS_SCENES else 'ogre'
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise ValueError('正在切换实验，请等待完成')
            self.phase, self.error = 'restarting', ''
            self.startup_failure = None
            self.tf_refreshed = False
            def run():
                try:
                    self._stop()
                    ident = 'experiment-'+uuid.uuid4().hex[:12]
                    folder = self.directory/ident
                    folder.mkdir()
                    write_json(folder/'software.json',dict(
                        software=getattr(self,'software',dict(verified=False)),
                        terrain_strategy=os.environ.get('MOONCAR_TERRAIN_STRATEGY','baseline'),
                        simulation_profile=task.get('simulation_profile','legacy'),
                        localization_mode=self.laser_odometry_mode if self.laser_odometry else None))
                    (folder/'task.json').write_text(json.dumps(task, indent=2)+'\n', encoding='utf-8')
                    (folder/'scene-setup.json').write_text(json.dumps(setup, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
                    survey_world(scene).write(folder/'survey_yard.sdf',encoding='unicode',xml_declaration=True)
                    x,y,yaw = task['start']
                    gui=ET.parse(WORLD).find('./world/gui')
                    gui.find("plugin[@filename='MinimalScene']/engine").text=setup['render_engine']
                    gui.find("plugin[@filename='MinimalScene']/camera_pose").text=f'{x-4} {y-5} {setup["initial_z"]+4.5} 0 0.55 0.90'
                    tree=gui.find("plugin[@filename='EntityTree']")
                    if tree is not None:gui.remove(tree)
                    config='<window><width>1100</width><height>740</height></window>\n'+''.join(ET.tostring(p,encoding='unicode') for p in gui)
                    (folder/'gui.config').write_text(config,encoding='utf-8')
                    env = dict(os.environ, ROS_DOMAIN_ID=self.ros_domain, GZ_PARTITION='mooncar-'+ident)
                    env['MOONCAR_WINDOW_OUTPUT']=str(folder/'window-localization.jsonl')
                    env['MOONCAR_TERRAIN_EVIDENCE_PATH']=str(folder/'terrain-cell-evidence.json')
                    if task.get('simulation_profile')=='r5_support_v1':
                        env['MOONCAR_SUPPORT_RECEIPT_ROOT']=str(folder/'support-runtime')
                    self.log = (folder/'simulation.log').open('w')
                    command = ['xvfb-run','-a','-s','-screen 0 1120x760x24','ros2','launch',str(ROOT/'containers/experiment.launch.py'),
                               f'initial_x:={x}',f'initial_y:={y}',f'initial_yaw:={yaw}', f'output_directory:={folder}',
                               f'initial_z:={setup["initial_z"]}', f'world_name:={setup["world_name"]}',
                               f'render_engine:={setup["render_engine"]}',
                               f'source_description:={setup["source_description"]}',
                               f"capture_gui:={'true' if task.get('gui',True) else 'false'}",
                               f"simulation_profile:={task.get('simulation_profile','legacy')}",
                               # Keep the producer/recorder selection identical to
                               # the workbench gate, even if the environment changes.
                               f"laser_odometry:={str(self.laser_odometry).lower()}",
                               f"laser_odometry_mode:={self.laser_odometry_mode}"]
                    process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                    with self.lock:
                        self.process, self.current = process, dict(task, id=ident)
                        self.started_at = time.monotonic()
                        self.phase = 'starting'
                except Exception as exc:
                    with self.lock:
                        self.phase, self.error = 'failed', str(exc)
            self.worker = threading.Thread(target=run, daemon=True)
            self.worker.start()
        return dict(accepted=True)

    def stop(self):
        with self.lock:
            if self.worker and self.worker.is_alive():
                raise ValueError('正在切换实验，请等待完成')
            self.phase = 'stopping'
            def run():
                try:
                    self._stop()
                    with self.lock:
                        self.phase = 'stopped'
                        self.worker = None
                except Exception as exc:
                    with self.lock:
                        self.phase, self.error = 'failed', str(exc)
            self.worker = threading.Thread(target=run, daemon=True)
            self.worker.start()
        return dict(accepted=True)

    def close(self):
        if self.worker:
            self.worker.join()
        self._stop()
        self.executor.shutdown();self.ros_thread.join(timeout=3)
        self.motion.destroy_node()
        import rclpy
        rclpy.shutdown()
