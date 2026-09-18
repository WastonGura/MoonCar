"""Two fresh live Gazebo trials: hold the fixed camera or approach the ramp through Nav2."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
from urllib.request import urlopen

from compare_survey_strategies import request,wait_for,read_pending_json
from survey_evaluation import write_json


def run(output,conditions=('stationary','approach'),goal=(5.9,-1.4),seconds=45,start=(6.5,-2.,2.4),scene=None):
    if len(goal)!=2 or not all(math.isfinite(v) for v in goal) or not 15<=seconds<=90:
        raise ValueError('Finite XY goal and 15..90 capture seconds required')
    import rclpy
    from rclpy.parameter import Parameter
    from simulation_experiment import validate_task
    task=validate_task(dict(version=1,region=[-25,-25,25,25],start=list(start),**({'scene':scene} if scene is not None else {})))
    state=request()
    if state.get('autonomy',{}).get('active') or state.get('motion',{}).get('active'):
        raise ValueError('Finish the active task first')
    output.mkdir(parents=True,exist_ok=False)
    write_json(output/'plan.json',dict(start=list(start),goal=list(goal),
               seconds=seconds,baseline_seconds=5,conditions=list(conditions),scene=task.get('scene','survey'),
               planned_path=[list(start[:2]),list(goal)],path_kind='Predeclared straight segment; not a queried Nav2 global path',
               limits='One paired mechanism trial; not strategy efficacy or physical validation'))
    for condition in conditions:
        old=(request().get('task') or {}).get('id')
        request('apply',task)
        state=wait_for(lambda s:(s.get('task') or {}).get('id') not in (None,old) and s.get('motion',{}).get('ready') and s.get('session'),240)
        ident=state['task']['id'];root=Path('/evidence/frontend-experiments')/ident
        env=dict(os.environ,ROS_DOMAIN_ID='89',GZ_PARTITION='mooncar-'+ident,OPENBLAS_NUM_THREADS='1')
        destination=output/condition
        log=(output/f'{condition}.log').open('w')
        process=subprocess.Popen([sys.executable,'/source/scripts/trajectory_evaluation.py','capture',str(destination),
             '--seconds',str(seconds),'--raw-depth','--world-sdf',str(root/'survey_yard.sdf'),
             '--robot-sdf','/source/reports/evidence/terrain-workbench/projection-20260913/fixed-motion/robot.sdf'],env=env,stdout=log,stderr=subprocess.STDOUT)
        rclpy.init();node=rclpy.create_node('slope_trial_clock',parameter_overrides=[Parameter('use_sim_time',value=True)])
        token=None;result=dict(condition=condition,experiment=ident,start_pose=state['motion']['pose'])
        first=None;started=False;deadline=time.monotonic()+600
        try:
            while process.poll() is None and time.monotonic()<deadline:
                rclpy.spin_once(node,timeout_sec=.1)
                manifest=destination/'manifest.json'
                if first is None:
                    value=read_pending_json(manifest) or {}
                    if value.get('status')=='recording':
                        first=node.get_clock().now().nanoseconds/1e9
                        try:(output/f'{condition}-start.jpg').write_bytes(urlopen('http://127.0.0.1:8766/api/simulation/scene.jpg',timeout=10).read())
                        except Exception as error:write_json(output/f'{condition}-start-scene-error.json',dict(error=str(error)))
                if first is not None and not started and node.get_clock().now().nanoseconds/1e9-first>=5:
                    started=True
                    result['action_started_sim_stamp']=node.get_clock().now().nanoseconds/1e9
                    if condition=='approach':token=request('move',dict(experiment=ident,goal=list(goal)))['token']
                if token:
                    if not request('heartbeat',dict(token=token)).get('active'):
                        result['motion_terminal_sim_stamp']=node.get_clock().now().nanoseconds/1e9
                        result['motion_terminal_phase']=request()['motion']['phase'];token=None
            if process.poll() is None:raise TimeoutError('Capture wall-time deadline exceeded')
            result['capture_exit']=process.returncode
        except Exception as error:result['error']=str(error)
        finally:
            result['motion_before_stop']=request()['motion']
            request('cancel',{})
            if process.poll() is None:process.terminate();process.wait(timeout=20)
            result['end_motion']=request()['motion'];result['status']='completed' if process.returncode==0 and 'error' not in result else 'failed'
            write_json(output/f'{condition}-control.json',result)
            try:(output/f'{condition}-end.jpg').write_bytes(urlopen('http://127.0.0.1:8766/api/simulation/scene.jpg',timeout=10).read())
            except Exception as error:write_json(output/f'{condition}-scene-error.json',dict(error=str(error)))
            node.destroy_node();rclpy.shutdown();log.close()
        print(json.dumps({k:result[k] for k in ('condition','experiment','status','capture_exit','error') if k in result},ensure_ascii=False),flush=True)
    request('stop',{})


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path)
    p.add_argument('--condition',choices=('stationary','approach'),action='append')
    p.add_argument('--goal',nargs=2,type=float,default=[5.9,-1.4]);p.add_argument('--seconds',type=float,default=45)
    p.add_argument('--start',nargs=3,type=float,default=[6.5,-2.,2.4])
    p.add_argument('--scene',choices=('survey','visible_ramp_v1','visible_curve_v1'))
    a=p.parse_args();run(a.output,a.condition or ('stationary','approach'),a.goal,a.seconds,a.start,a.scene)
