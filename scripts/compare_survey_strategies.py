"""Run a predeclared, reset-per-trial strategy comparison with mapping acceptance evidence."""
import argparse
import errno
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import warnings
from urllib.request import Request, urlopen
from urllib.error import HTTPError

from autonomous_survey import LEGACY_STRATEGIES,STRATEGIES
from survey_evaluation import write_json, write_csv

ROOT=Path(__file__).resolve().parents[1]
DEFAULT_ROBOT_SDF=(
    ROOT/'ros2_ws/src/mooncar_description/urdf/robot.sdf'
    if (ROOT/'ros2_ws/src/mooncar_description/urdf/robot.sdf').is_file()
    else ROOT/'reports/evidence/terrain-workbench/projection-20260913/fixed-motion/robot.sdf'
)


def read_pending_json(path):
    """A polling reader can race an atomic replacement on the Windows bind mount."""
    try:return json.loads(path.read_text(encoding='utf-8'))
    except (FileNotFoundError,json.JSONDecodeError):return None
    except OSError as error:
        if error.errno!=getattr(errno,'ENODATA',61):raise
        warnings.warn(f'Transient pending JSON read: {path}: {error}',RuntimeWarning)
        return None


def request(action='', body=None):
    url='http://127.0.0.1:8766/api/simulation'+('/'+action if action else '')
    req=Request(url,data=json.dumps(body).encode() if body is not None else None,
                headers={'Content-Type':'application/json','Origin':'http://127.0.0.1:8766'})
    try:
        with urlopen(req,timeout=15) as response:return json.load(response)
    except HTTPError as error:
        raise ValueError(f'HTTP {error.code}: {error.read(2048).decode("utf-8",errors="replace")}') from error


def wait_for(predicate, seconds):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        state=request()
        if predicate(state):return state
        time.sleep(1)
    raise TimeoutError('Workbench state deadline exceeded')


def capture_command(folder,world,ident,budget,robot_sdf):
    stop_file=folder/'stop.request'
    command=[sys.executable,str(ROOT/'scripts/trajectory_evaluation.py'),'capture',str(folder),
             '--seconds',str(budget),'--raw-depth','--runtime-contract','--stop-file',str(stop_file),
             '--world-sdf',str(world),'--robot-sdf',str(robot_sdf)]
    env=dict(os.environ,ROS_DOMAIN_ID='89',GZ_PARTITION='mooncar-'+ident,OPENBLAS_NUM_THREADS='1')
    if Path('/opt/ros/jazzy/setup.bash').exists():
        env.setdefault('AMENT_PREFIX_PATH', '/opt/ros/jazzy')
        env.setdefault('ROS_DISTRO', 'jazzy')
        py_path = env.get('PYTHONPATH', '')
        if '/opt/ros/jazzy/lib/python3.12/site-packages' not in py_path:
            env['PYTHONPATH'] = f"/opt/ros/jazzy/lib/python3.12/site-packages:{py_path}".strip(':')
    if Path('/work/ros2_ws/install/mooncar_navigation').exists():
        ws_inst = '/work/ros2_ws/install'
        pkgs = [
            f'{ws_inst}/mooncar_navigation',
            f'{ws_inst}/mooncar_terrain',
            f'{ws_inst}/mooncar_slam',
            f'{ws_inst}/mooncar_msgs',
            f'{ws_inst}/mooncar_gazebo',
            f'{ws_inst}/mooncar_description',
            f'{ws_inst}/mooncar_bringup',
            f'{ws_inst}/mooncar_control',
        ]
        curr_ament = env.get('AMENT_PREFIX_PATH', '')
        missing = [p for p in pkgs if p not in curr_ament]
        if missing:
            env['AMENT_PREFIX_PATH'] = ':'.join(missing + ([curr_ament] if curr_ament else []))
    return command,env,stop_file


def wait_for_capture(folder,process,seconds=60):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        manifest=read_pending_json(folder/'manifest.json') or {}
        if manifest.get('status')=='recording':return manifest
        if process.poll() is not None:raise RuntimeError('Synchronized raw-depth capture did not start')
        time.sleep(.5)
    raise TimeoutError('Synchronized raw-depth capture startup deadline exceeded')


def run(output, budget=90, repeats=2, starts=None, strategies=STRATEGIES, scene='survey', robot_sdf=DEFAULT_ROBOT_SDF):
    if not 15<=budget<=300 or not 1<=repeats<=10:raise ValueError('Budget 15..300 and repeats 1..10 required')
    if not strategies or len(set(strategies))!=len(strategies) or any(s not in (*LEGACY_STRATEGIES,*STRATEGIES) for s in strategies):
        raise ValueError('Choose distinct supported strategies')
    if os.environ.get('ROS_DOMAIN_ID')!='89':raise ValueError('Runner and capture require ROS_DOMAIN_ID=89')
    if not robot_sdf.is_file():raise ValueError('Reviewed converted robot SDF is required')
    state=request()
    if state.get('motion',{}).get('active') or state.get('autonomy',{}).get('active'):
        raise ValueError('An active movement/task exists; finish it before this comparison')
    starts=starts or [[3.5,3.,0.],[0.,-3.,0.]]
    from report_mapping_acceptance import prepare,evaluate
    from simulation_experiment import validate_task,survey_world
    tasks=[validate_task(dict(version=1,region=[-25,-25,25,25],start=start,scene=scene,gui=False)) for start in starts]
    output.mkdir(parents=True,exist_ok=False)
    references=output/'references';references.mkdir()
    for index,task in enumerate(tasks,1):
        world=references/f'start-{index:02d}-world.sdf'
        survey_world(scene).write(world,encoding='unicode',xml_declaration=True)
        prepare(world,references/f'start-{index:02d}',task['start'][:2],8,budget)
    plan=[dict(start=task['start'],start_index=i+1,scene=scene,capture_gui=False,repeat=repeat+1,
               strategy=strategies[(j+repeat+i)%len(strategies)],budget_seconds=budget,radius=8)
          for repeat in range(repeats) for i,task in enumerate(tasks) for j in range(len(strategies))]
    write_json(output/'plan.json',dict(trials=plan,reset='new Gazebo process and empty recorder/SLAM per trial',
               baseline='first synchronized raw-depth frame; actual early endings retained',
               evaluation='mapping-acceptance-v1.1; references frozen before every trial',
               purpose='predeclared strategy comparison; repeats and scenes must be fixed before interpreting results'))
    for name in ('captures','evaluations','logs'):(output/name).mkdir()
    results=[]
    for number,trial in enumerate(plan,1):
        row=dict(trial,index=number,status='starting');results.append(row)
        write_json(output/'trials.json',results)
        print(f'Trial {number}/{len(plan)} {trial}',flush=True)
        old=None;applied=False;capture_process=None;capture_log=None;stop_file=None
        try:
            old=(request().get('task') or {}).get('id')
            request('apply',tasks[trial['start_index']-1]);applied=True
            state=wait_for(lambda s:(s.get('task') or {}).get('id') not in (None,old)
                           and s.get('motion',{}).get('ready') and s.get('session'),240)
            ident=state['task']['id'];row['experiment']=ident
            trial_tag=f"trial-{number:02d}-{trial['strategy']}"
            capture=output/'captures'/trial_tag
            world=Path('/evidence/frontend-experiments')/ident/'survey_yard.sdf'
            command,env,stop_file=capture_command(capture,world,ident,budget,robot_sdf)
            capture_log=(output/'logs'/f'{trial_tag}-capture.log').open('w')
            capture_process=subprocess.Popen(command,env=env,stdout=capture_log,stderr=subprocess.STDOUT)
            manifest=wait_for_capture(capture,capture_process)
            request('explore',dict(experiment=ident,budget_seconds=budget,radius=8,strategy=trial['strategy']))
            state=wait_for(lambda s: bool(s.get('autonomy',{}).get('archive')),20)
            archive=state['autonomy']['archive'];row['archive']=archive
            state=wait_for(lambda s: s.get('autonomy',{}).get('archive')==archive and not s['autonomy']['active'],max(2700, int(budget * 9)))
            row['status']=state['autonomy']['phase'];row['reason']=state['autonomy']['reason']
            autonomy_started=state['autonomy']['started']
            stop_file.touch()
            if capture_process.wait(timeout=120):raise RuntimeError('Synchronized raw-depth capture failed')
            capture_log.close();capture_log=None;row['capture']=str(capture)
            final_manifest=read_pending_json(capture/'manifest.json') or {}
            row['capture_lead_seconds']=autonomy_started-final_manifest['first_stamp']
            evidence=output/'evaluations'/trial_tag
            acceptance=evaluate(capture,references/f"start-{trial['start_index']:02d}",evidence)
            row['evaluation']=str(evidence)
            row['acceptance']={k:acceptance.get(k) for k in ('status','S','C_obs','C','P','height_rmse_m',
                'slope_mae_deg','baseline_S','delta_S','evaluation_area_m2')}
            folder=Path('/evidence/frontend-experiments')/ident/'research'/Path(archive).stem
            row['folder']=str(folder)
            deadline=time.monotonic()+180
            while time.monotonic()<deadline:
                if (read_pending_json(folder/'manifest.json') or {}).get('report_status') in ('ready','failed'):break
                time.sleep(1)
            if (folder/'summary.json').exists():row['summary']=json.loads((folder/'summary.json').read_text())
            else:row['report_error']='Legacy survey report missing at deadline'
        except Exception as error:
            row.update(status='trial_failed',error=str(error))
            try:
                state=request();task=state.get('task') or {}
                if applied and task.get('id') not in (None,old):
                    row['experiment']=task['id'];write_json(output/f'{trial_tag}-failure-state.json',state)
                request('explore_stop',{})
            except Exception as diagnostic_error:
                row['diagnostic_error']=str(diagnostic_error);write_json(output/'trials.json',results)
                raise
        finally:
            if stop_file is not None and capture_process is not None and capture_process.poll() is None:
                if stop_file.parent.exists():stop_file.touch()
                try:capture_process.wait(timeout=30)
                except subprocess.TimeoutExpired:capture_process.terminate();capture_process.wait(timeout=20)
            if capture_log is not None:capture_log.close()
        write_json(output/'trials.json',results)
        print(json.dumps({k:v for k,v in row.items() if k not in ('summary','acceptance')},ensure_ascii=False),flush=True)
    request('stop',{})
    state=wait_for(lambda s:s.get('phase') in ('stopped','failed'),120)
    if state['phase']=='failed':raise RuntimeError(f"Experiment stop failed: {state.get('error')}")
    write_csv(output/'comparison.csv',[{**{k:v for k,v in r.items() if k not in ('summary','acceptance')},
                   **r.get('summary',{}),**{'acceptance_'+k:v for k,v in r.get('acceptance',{}).items()},
                   'start':json.dumps(r['start'])} for r in results])
    if any(r['status']=='trial_failed' for r in results):raise RuntimeError('One or more strategy trials failed; see trials.json')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path);parser.add_argument('--budget',type=int,default=90)
    parser.add_argument('--repeats',type=int,default=2)
    parser.add_argument('--start',type=float,nargs=3,action='append')
    parser.add_argument('--strategy',choices=(*LEGACY_STRATEGIES,*STRATEGIES),action='append')
    parser.add_argument('--scene',choices=('survey','visible_ramp_v1','visible_curve_v1'),default='survey')
    parser.add_argument('--robot-sdf',type=Path,default=DEFAULT_ROBOT_SDF)
    args=parser.parse_args();run(args.output,args.budget,args.repeats,args.start,args.strategy or STRATEGIES,args.scene,args.robot_sdf)
