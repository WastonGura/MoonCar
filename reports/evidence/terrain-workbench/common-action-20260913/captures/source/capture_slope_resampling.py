"""Two fresh live Gazebo trials: hold the fixed camera or approach the ramp through Nav2."""
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback
from urllib.request import urlopen

from compare_survey_strategies import request,wait_for,read_pending_json
from survey_evaluation import write_json


def run(output,conditions=('stationary','approach'),goal=(5.9,-1.4),seconds=45,start=(6.5,-2.,2.4),scene=None,nav2_plan=False,observation_yaw=0.,observe_turn=True,arrival_wait=0.,observation_delta=None,skip_probe=False,targeted_probe=False,comparison_action=None):
    if comparison_action is not None:
        from compare_observation_actions import validate_action
        validate_action(comparison_action)
        if not nav2_plan or skip_probe or targeted_probe or observation_delta is not None or not observe_turn or tuple(conditions)!=('approach',):
            raise ValueError('Action comparison requires approach/Nav2 with no other observation selector')
    if targeted_probe:skip_probe=True
    if len(goal)!=2 or not all(math.isfinite(v) for v in goal) or not 15<=seconds<=90:
        raise ValueError('Finite XY goal and 15..90 capture seconds required')
    if not math.isfinite(observation_yaw) or abs(observation_yaw)>math.pi:raise ValueError('Finite observation yaw within +/-pi required')
    if not math.isfinite(arrival_wait) or not 0<=arrival_wait<=5:raise ValueError('Arrival wait must be 0..5 simulation seconds')
    if observation_delta is not None and (not nav2_plan or not math.isfinite(observation_delta) or abs(observation_delta)>math.pi):
        raise ValueError('Relative observation angle requires Nav2 planning and a finite angle within pi')
    if skip_probe and (not nav2_plan or observation_delta is not None):
        raise ValueError('Skip probe requires Nav2 and its frozen angle selection, not observation_delta')
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
               nav2_plan=nav2_plan,observe_turn=observe_turn,arrival_wait=arrival_wait,observation_delta=observation_delta,skip_probe=skip_probe,targeted_probe=targeted_probe,observation_yaw=observation_yaw if nav2_plan else None,observation_tolerance_rad=.1 if nav2_plan else None,
               comparison_action=comparison_action,limits='One paired mechanism trial; not strategy efficacy or physical validation'))
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
        token=None;stage='move';result=dict(condition=condition,experiment=ident,start_pose=state['motion']['pose'])
        first=None;started=False;pending_observation=None;target_yaw=observation_yaw;deadline=time.monotonic()+600
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
                    if condition=='approach' and nav2_plan:
                        if targeted_probe or comparison_action is not None:
                            from skip_probe import freeze_targets
                            result['frozen_targets']=freeze_targets(destination,node)
                        from observation_path import compute_path
                        planned=compute_path(node,list(goal),request(),observation_yaw)
                        write_json(output/f'{condition}-nav2-plan.json',planned)
                        result['nav2_plan_file']=f'{condition}-nav2-plan.json'
                    result['action_started_sim_stamp']=node.get_clock().now().nanoseconds/1e9
                    if condition=='approach':token=request('move',dict(experiment=ident,goal=list(goal)))['token']
                if token:
                    if not request('heartbeat',dict(token=token)).get('active'):
                        result['motion_terminal_sim_stamp']=node.get_clock().now().nanoseconds/1e9
                        terminal=request()['motion'];result['motion_terminal_phase']=terminal['phase'];token=None
                        if nav2_plan:
                            result[stage+'_terminal']=dict(phase=terminal['phase'],pose=terminal['pose'],error=terminal['error'],sim_stamp=result['motion_terminal_sim_stamp'])
                            if stage=='move' and terminal['phase']=='succeeded':
                                pending_observation=result['motion_terminal_sim_stamp']+arrival_wait
                            if stage=='compare_move' and terminal['phase']=='succeeded':
                                stage='observe_turn';token=request('turn',dict(experiment=ident,yaw=target_yaw))['token']
                if pending_observation is not None and node.get_clock().now().nanoseconds/1e9>=pending_observation:
                    if comparison_action is not None:
                        from compare_observation_actions import prepare
                        action=prepare(destination,node,comparison_action,seconds)
                        result['comparison']=action
                        result['observation_feature_stamp']=action['feature_stamp']
                        result['observation_decision_pose']=action['pose']
                        result['observation_decision_sim_stamp']=action['decision_stamp']
                        result['observation_target_yaw']=target_yaw=action['yaw']
                        pending_observation=None
                        if action['arm']=='advance':
                            stage='compare_move';token=request('move',dict(experiment=ident,goal=action['goal']))['token']
                        elif action['arm']!='hold':
                            stage='observe_turn';token=request('turn',dict(experiment=ident,yaw=target_yaw))['token']
                        continue
                    selected=None
                    if skip_probe:
                        from skip_probe import select_live
                        selected=select_live(destination,node,seconds,targeted=targeted_probe)
                        result['skip_probe']=selected
                        result['observation_feature_stamp']=selected['feature_stamp']
                    motion=request()['motion'];pose=motion['pose']
                    if selected and selected['selected'] is not None:
                        target_yaw=selected['selected']['yaw']
                    if observation_delta is not None:
                        target_yaw=math.atan2(math.sin(pose['yaw']+observation_delta),math.cos(pose['yaw']+observation_delta))
                    result['observation_decision_pose']=pose
                    result['observation_target_yaw']=target_yaw
                    result['observation_decision_sim_stamp']=motion['diagnostics']['sim_stamp'] if skip_probe else node.get_clock().now().nanoseconds/1e9
                    pending_observation=None
                    if observe_turn and (not skip_probe or selected['executable']):
                        stage='observe_turn';token=request('turn',dict(experiment=ident,yaw=target_yaw))['token']
            if process.poll() is None:raise TimeoutError('Capture wall-time deadline exceeded')
            result['capture_exit']=process.returncode
        except Exception as error:
            result.update(error=str(error),error_traceback=traceback.format_exc(),error_stage=stage)
        finally:
            result['motion_before_stop']=request()['motion']
            request('cancel',{})
            if process.poll() is None:process.terminate();process.wait(timeout=20)
            result['end_motion']=request()['motion'];result['status']='completed' if process.returncode==0 and 'error' not in result else 'failed'
            if nav2_plan and observe_turn and condition=='approach':
                terminal=result.get('observe_turn_terminal',{});pose=terminal.get('pose') or {}
                error=math.atan2(math.sin(pose['yaw']-target_yaw),math.cos(pose['yaw']-target_yaw)) if 'yaw' in pose else None
                result['observation_yaw_error_rad']=error
                if 'yaw' in pose and result.get('observation_decision_pose'):
                    before=result['observation_decision_pose']['yaw']
                    result['observation_actual_delta']=math.atan2(math.sin(pose['yaw']-before),math.cos(pose['yaw']-before))
                    if skip_probe:
                        result['observation_requested_delta']=math.atan2(math.sin(target_yaw-before),math.cos(target_yaw-before))
                result['observation_reached']=terminal.get('phase')=='succeeded' and error is not None and abs(error)<=.1
            write_json(output/f'{condition}-control.json',result)
            try:(output/f'{condition}-end.jpg').write_bytes(urlopen('http://127.0.0.1:8766/api/simulation/scene.jpg',timeout=10).read())
            except Exception as error:write_json(output/f'{condition}-scene-error.json',dict(error=str(error)))
            node.destroy_node();rclpy.try_shutdown();log.close()
        print(json.dumps({k:result[k] for k in ('condition','experiment','status','capture_exit','error') if k in result},ensure_ascii=False),flush=True)
    request('stop',{})


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('output',type=Path)
    p.add_argument('--condition',choices=('stationary','approach'),action='append')
    p.add_argument('--goal',nargs=2,type=float,default=[5.9,-1.4]);p.add_argument('--seconds',type=float,default=45)
    p.add_argument('--start',nargs=3,type=float,default=[6.5,-2.,2.4])
    p.add_argument('--scene',choices=('survey','visible_ramp_v1','visible_curve_v1'))
    p.add_argument('--nav2-plan',action='store_true');p.add_argument('--observation-yaw',type=float,default=0.)
    p.add_argument('--no-observe-turn',action='store_true',help='Hold the arrival heading for a same-budget control')
    p.add_argument('--arrival-wait',type=float,default=0.,help='Common pre-decision settling window in simulation seconds')
    p.add_argument('--observation-delta',type=float,help='Relative to the measured decision heading; resolve equally in both arms')
    a=p.parse_args();run(a.output,a.condition or ('stationary','approach'),a.goal,a.seconds,a.start,a.scene,a.nav2_plan,a.observation_yaw,not a.no_observe_turn,a.arrival_wait,a.observation_delta)
