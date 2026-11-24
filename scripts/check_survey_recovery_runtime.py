"""Run isolated, explicitly injected recovery acceptance and one natural mission.

The test planner supplies path-validated fixtures in controlled trials. Only the
normal trial uses choose_view unmodified. Injected failures are never field data.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]


def write(path, value):
    from survey_evaluation import write_json
    write_json(path, value)


def wait(predicate, seconds, label):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        value = predicate()
        if value:
            return value
        time.sleep(.1)
    raise TimeoutError(label)


def future_value(future, seconds=20):
    wait(future.done, seconds, 'ROS action response timeout')
    return future.result()


def yaw(q):
    return math.atan2(2*(q['w']*q['z']+q['x']*q['y']),
                      1-2*(q['y']**2+q['z']**2))


def angle(a, b):
    return abs(math.atan2(math.sin(a-b), math.cos(a-b)))


def make_proxy(failures):
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose, ComputePathToPose
    from rclpy.action import ActionClient, ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node

    class Proxy(Node):
        def __init__(self):
            super().__init__('mooncar_recovery_test_proxy')
            group = ReentrantCallbackGroup()
            self.calls = []
            self.inner = {}
            self.client = ActionClient(self, NavigateToPose, '/navigate_to_pose', callback_group=group)
            self.planner = ActionClient(self, ComputePathToPose, '/compute_path_to_pose', callback_group=group)
            self.server = ActionServer(self, NavigateToPose, '/mooncar/recovery_test/navigate_to_pose',
                                       execute_callback=self.execute, cancel_callback=self.cancel,
                                       callback_group=group)

        def cancel(self, goal):
            handle = self.inner.get(bytes(goal.goal_id.uuid))
            if handle is not None:
                handle.cancel_goal_async()
            return CancelResponse.ACCEPT

        async def execute(self, goal):
            index = len(self.calls)+1
            row = dict(index=index, injected=index <= failures, wall=time.monotonic(),
                       x=goal.request.pose.pose.position.x, y=goal.request.pose.pose.position.y)
            self.calls.append(row)
            if row['injected']:
                result = NavigateToPose.Result()
                result.error_code = 208
                result.error_msg = 'TEST INJECTION: declared no-path result; no chassis command'
                row.update(status=GoalStatus.STATUS_ABORTED, error_code=208)
                goal.abort()
                return result
            handle = await self.client.send_goal_async(goal.request,
                                feedback_callback=lambda msg: goal.publish_feedback(msg.feedback))
            if not handle.accepted:
                row.update(status=GoalStatus.STATUS_ABORTED, error='real Nav2 rejected')
                goal.abort()
                result = NavigateToPose.Result()
                result.error_code = 208
                result.error_msg = 'Real Nav2 rejected the forwarded goal'
                return result
            self.inner[bytes(goal.goal_id.uuid)] = handle
            if goal.is_cancel_requested:
                handle.cancel_goal_async()
            response = await handle.get_result_async()
            self.inner.pop(bytes(goal.goal_id.uuid), None)
            row.update(status=response.status, error_code=response.result.error_code)
            if response.status == GoalStatus.STATUS_SUCCEEDED:
                goal.succeed()
            elif response.status == GoalStatus.STATUS_CANCELED:
                goal.canceled()
            else:
                goal.abort()
            return response.result

    return Proxy()


def make_witness(folder, partition):
    import numpy as np
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import JointState, Image
    from std_msgs.msg import String

    class Witness(Node):
        def __init__(self):
            super().__init__('mooncar_recovery_independent_witness')
            self.rows = []
            self.errors = []
            self.lock = threading.Lock()
            self.log = (folder/'witness.jsonl').open('w')
            self.gzlog = (folder/'truth-stderr.log').open('w')
            self.create_subscription(JointState, '/joint_states', self.joints, qos_profile_sensor_data)
            self.create_subscription(Twist, '/mooncar/cmd_vel_safe', self.command, 10)
            self.create_subscription(Image, '/mooncar/depth/image', self.depth, qos_profile_sensor_data)
            self.create_subscription(String, '/mooncar/map/quality', self.quality, 1)
            self.process = subprocess.Popen(['gz','topic','-e','-t',
                       '/world/lunar_yard/dynamic_pose/info','--json-output'],
                       stdout=subprocess.PIPE, stderr=self.gzlog, text=True,
                       env=dict(os.environ, GZ_PARTITION=partition))
            self.thread = threading.Thread(target=self.truth, daemon=True)
            self.thread.start()

        def record(self, kind, **data):
            row = dict(kind=kind, wall=time.monotonic(),
                       sim=self.get_clock().now().nanoseconds/1e9, **data)
            with self.lock:
                self.rows.append(row)
                self.log.write(json.dumps(row, allow_nan=False)+'\n')
            return row

        def joints(self, msg):
            positions = {n:float(p) for n,p in zip(msg.name,msg.position)
                         if n in ('camera_pan_joint','camera_tilt_joint')}
            if len(positions) == 2 and np.isfinite(list(positions.values())).all():
                self.record('joints', **positions)

        def command(self, msg):
            values = [msg.linear.x,msg.linear.y,msg.linear.z,
                      msg.angular.x,msg.angular.y,msg.angular.z]
            self.record('safe_command', finite=bool(np.isfinite(values).all()),
                        zero=bool(np.isfinite(values).all() and max(map(abs,values)) <= 1e-6),
                        values=[float(v) if math.isfinite(v) else None for v in values])

        def depth(self, msg):
            valid = 0
            if msg.encoding == '32FC1':
                values = np.frombuffer(msg.data, dtype='>f4' if msg.is_bigendian else '<f4')
                valid = int((np.isfinite(values) & (values > 0)).sum())
            self.record('depth', encoding=msg.encoding, finite_returns=valid)

        def quality(self, msg):
            data = json.loads(msg.data)
            self.record('quality', frames=data.get('frames'), stamp=data.get('stamp'))

        def truth(self):
            try:
                for line in self.process.stdout:
                    if not line.strip():
                        continue
                    data = json.loads(line)
                    body = next((p for p in data['pose'] if p.get('name') == 'mooncar'), None)
                    if body is None:
                        continue
                    p, q = body.get('position',{}), body.get('orientation',{})
                    q = {k:float(q.get(k, 1 if k == 'w' else 0)) for k in ('x','y','z','w')}
                    self.record('truth', x=float(p.get('x',0)), y=float(p.get('y',0)), yaw=yaw(q))
            except Exception as error:
                self.errors.append(str(error))

        def close(self):
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill(); self.process.wait()
            self.thread.join(timeout=5)
            self.log.close(); self.gzlog.close()

    return Witness()


def fixtures(motion, proxy, folder):
    from nav2_msgs.action import ComputePathToPose
    candidates = []
    rejected = []
    for distance in (1., 1.3, 1.6, 1.9):
        for bearing in (0., -.35, .35, -.7, .7):
            x, y = distance*math.cos(bearing), distance*math.sin(bearing)
            try:
                with motion.lock:
                    motion.check_goal([x,y])
                request = ComputePathToPose.Goal()
                request.planner_id = 'GridBased'
                request.goal.header.frame_id = 'map'
                request.goal.pose.position.x = x; request.goal.pose.position.y = y
                request.goal.pose.orientation.w = 1.
                handle = future_value(proxy.planner.send_goal_async(request))
                if not handle.accepted:
                    raise ValueError('planner rejected')
                response = future_value(handle.get_result_async())
                if response.status != 4 or response.result.error_code or len(response.result.path.poses) < 2:
                    raise ValueError('no real Nav2 path')
                candidates.append(dict(x=x,y=y,yaw=0.,kind='advance',
                    expected_new_m2=0.,expected_improvement_m2=0.,expected_utility_m2=0.,
                    visible_cells=[],estimated_seconds=distance/.12+6,
                    test_fixture=True,path_validation=dict(status=response.status,
                        error_code=response.result.error_code,poses=len(response.result.path.poses))))
                if len(candidates) == 6:
                    write(folder/'fixtures.json', dict(candidates=candidates,rejected=rejected))
                    return candidates
            except (ValueError, TimeoutError) as error:
                rejected.append(dict(x=x,y=y,error=str(error)))
    write(folder/'fixtures.json', dict(candidates=candidates,rejected=rejected))
    raise RuntimeError('Six distinct observed, real-path-validated fixtures unavailable')


def commands_zero(rows):
    return len(rows)>=5 and all(len(r.get('values',[]))==6 and
        all(type(v) in (float,int) and math.isfinite(v) and abs(v)<=1e-6 for v in r['values'])
        for r in rows)


def evaluate(kind, state, rows, calls, errors):
    recovery = [r for r in state['history'] if r.get('recovery')]
    checks = dict(witness_clean=not errors, autonomous_inactive=not state['active'])
    metrics = {}
    if kind != 'natural':
        navigation = [r for r in state['history'] if not r.get('recovery')]
        checks.update(one_recovery=len(recovery)==1 and state['recovery_attempts']==1,
                      injection_count=sum(c['injected'] for c in calls)==(3 if kind=='success' else 6),
                      three_failures_before_scan=bool(len(state['history'])>=4 and
                          all(not r.get('success') and not r.get('recovery') for r in state['history'][:3])
                          and state['history'][3].get('recovery')),
                      failed_targets_retained=all(r.get('retry_blocked') for r in navigation if not r.get('success')))
        begin = next((r for r in rows if r['kind']=='recovery_begin'), None)
        end = next((r for r in rows if r['kind']=='recovery_end'), None)
        checks['recovery_interval_present'] = begin is not None and end is not None
        if begin and end:
            interval = [r for r in rows if begin['wall'] <= r['wall'] <= end['wall']]
            truths = [r for r in interval if r['kind']=='truth']
            joints = [r for r in interval if r['kind']=='joints']
            commands = [r for r in interval if r['kind']=='safe_command']
            qualities = [r for r in interval if r['kind']=='quality' and r['frames'] is not None]
            depths = [r for r in interval if r['kind']=='depth' and r['finite_returns']>=100]
            metrics['translation_m'] = max((math.hypot(p['x']-truths[0]['x'],p['y']-truths[0]['y']) for p in truths),default=None)
            metrics['yaw_change_rad'] = max((angle(p['yaw'],truths[0]['yaw']) for p in truths),default=None)
            settled = [r for r in joints if abs(r['camera_pan_joint']-math.pi/2)<=math.radians(5)
                       and abs(r['camera_tilt_joint']+.2618)<=math.radians(5)]
            metrics['joint_samples_at_target'] = len(settled)
            metrics['new_frames'] = (max(r['frames'] for r in qualities)-begin['frames']) if qualities else None
            checks.update(stationary=bool(truths) and metrics['translation_m']<=.05 and metrics['yaw_change_rad']<=.05,
                          gimbal_at_target=len(settled)>=2,
                          zero_during_recovery=commands_zero(commands),
                          new_keyframes=bool(qualities) and metrics['new_frames']>0,
                          valid_depth=bool(depths), recovery_succeeded=bool(recovery and recovery[0]['success']))
            # Do not accept a timer-based success preceding measured joint arrival.
            checks['joint_arrival_before_action_success'] = bool(settled) and settled[0]['wall']<=end['action_success_wall']
        if kind == 'fault':
            checks.update(fault_stop=state['phase']=='fault', no_seventh_navigation=len(calls)==6)
        else:
            actual = [c for c in calls if not c['injected']]
            checks['real_navigation_succeeded'] = bool(actual) and actual[-1].get('status')==4 and actual[-1].get('error_code')==0
            before = [r for r in rows if r['kind']=='truth' and begin and r['wall']<=begin['wall']]
            after = [r for r in rows if r['kind']=='truth' and end and r['wall']>end['wall']]
            metrics['real_navigation_displacement_m'] = max((math.hypot(p['x']-before[-1]['x'],p['y']-before[-1]['y']) for p in after),default=None) if before else None
            checks['real_displacement'] = metrics['real_navigation_displacement_m'] is not None and metrics['real_navigation_displacement_m']>=.5
    else:
        end = next((r for r in rows if r['kind']=='observation_end'),None)
        metrics['mission_elapsed_seconds'] = state['elapsed_seconds']
        metrics['observation_window_seconds'] = end['sim']-state['started'] if end else None
        checks['normal_budget_stop'] = state['phase']=='budget_reached'
        checks['budget_window_observed'] = bool(end) and metrics['observation_window_seconds']>=90.
        checks['actions_present'] = bool(state['history'])
    final = next((r for r in rows if r['kind']=='mission_end'), None)
    stopped = [r for r in rows if final and r['wall']>=final['wall'] and r['kind']=='safe_command']
    checks['zero_after_stop'] = commands_zero(stopped)
    checks['stop_observation_two_seconds'] = bool(stopped) and stopped[-1]['wall']-final['wall']>=2.
    return dict(passed=all(checks.values()), checks=checks, metrics=metrics,
                injected=kind!='natural', planner='path-validated test fixtures' if kind!='natural' else 'unmodified choose_view',
                final_phase=state['phase'], reason=state['reason'], witness_errors=errors)


def trial(folder, kind):
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    import autonomous_survey
    from simulation_experiment import SimulationExperiment
    folder.mkdir(parents=True, exist_ok=False)
    (folder/'sessions').mkdir()
    experiment = SimulationExperiment(folder/'sessions')
    proxy = witness = None
    witness_executor = witness_thread = None
    report = dict(passed=False, trial=kind)
    try:
        experiment.apply(dict(version=1,region=[-25,-25,25,25],start=[0,0,0],scene='survey',gui=True))
        def ready():
            state = experiment.state()
            if state['phase']=='failed':
                raise RuntimeError(state['error'])
            return state if state['motion'].get('ready') and state['session'] else None
        state = wait(ready,240,'fresh simulation readiness')
        partition = 'mooncar-'+state['task']['id']
        write(folder/'trial-plan.json',dict(trial=kind,domain=90,partition=partition,
              strategy='surface_view_v3',budget_seconds=90 if kind=='natural' else 120,radius=8,
              injected_navigation_failures=0 if kind=='natural' else (3 if kind=='success' else 6)))
        witness = make_witness(folder, partition)
        if getattr(witness,'dedicated_executor',False):
            from rclpy.executors import MultiThreadedExecutor
            witness_executor=MultiThreadedExecutor(num_threads=2)
            witness_executor.add_node(witness)
            witness_thread=threading.Thread(target=witness_executor.spin,daemon=True)
            witness_thread.start()
        else:experiment.executor.add_node(witness)
        wait(lambda: all(any(r['kind']==k for r in witness.rows) for k in ('truth','joints','depth','quality','safe_command')),
             30, 'independent witness topics')
        decisions = None
        if kind != 'natural':
            proxy = make_proxy(3 if kind=='success' else 6); experiment.executor.add_node(proxy)
            wait(lambda: proxy.client.server_is_ready() and proxy.planner.server_is_ready(),30,'real Nav2 services')
            decisions = fixtures(experiment.motion,proxy,folder)
            experiment.motion.client.destroy()
            experiment.motion.client = ActionClient(experiment.motion,NavigateToPose,'/mooncar/recovery_test/navigate_to_pose')
            wait(experiment.motion.client.server_is_ready,10,'test action proxy')
        survey = experiment.autonomy
        original_execute = survey.execute
        def execute(decision, turn=False):
            if decision.get('recovery'):
                witness.record('recovery_begin',frames=experiment.motion.quality['frames'])
                result = original_execute(decision, turn)
                witness.record('recovery_action_result',success=result)
                return result
            return original_execute(decision, turn)
        survey.execute = execute
        cursor = 0
        def fixture_planner(*args, **kwargs):
            nonlocal cursor
            limit = 4 if kind=='success' else 6
            order = [1,2,3,0] if kind=='success' else list(range(6))
            decision = dict(decisions[order[cursor]]) if cursor < limit else None
            cursor += 1
            return decision,dict(budget_limited=False,test_fixture=True)
        planner_context = patch.object(autonomous_survey,'choose_view',side_effect=fixture_planner) if decisions else patch.dict(os.environ,{})
        with planner_context:
            experiment.command('explore',dict(experiment=state['task']['id'],budget_seconds=90 if kind=='natural' else 120,
                                             radius=8,strategy='surface_view_v3'))
            deadline = time.monotonic()+900
            recovery_recorded = False
            while survey.state()['active'] and time.monotonic()<deadline:
                recovery = next((r for r in survey.state()['history'] if r.get('recovery')),None)
                if recovery and recovery.get('outcome')!='running' and not recovery_recorded:
                    result = next((r for r in witness.rows if r['kind']=='recovery_action_result'),None)
                    witness.record('recovery_end', action_success_wall=result['wall'] if result else 0.)
                    recovery_recorded = True
                time.sleep(.05)
            if survey.state()['active']:
                survey.stop('Test wall-clock deadline exceeded','fault')
            survey.worker.join(timeout=180)
        state = survey.state()
        witness.record('mission_end',phase=state['phase'],mission_elapsed_seconds=state['elapsed_seconds'])
        if kind=='natural':
            # Candidate budget guards may end the mission before 90 s. Preserve that
            # actual end and observe the parked rover for the remainder of the window.
            wait(lambda: survey.now()-state['started']>=90.,max(0.,deadline-time.monotonic()),
                 '90 simulation-second natural observation window')
        time.sleep(2.2)
        witness.record('observation_end')
        write(folder/'autonomy.json', state)
        write(folder/'proxy-calls.json', proxy.calls if proxy else [])
        report = evaluate(kind,state,list(witness.rows),proxy.calls if proxy else [],witness.errors)
        report.update(trial=kind,experiment=experiment.current['id'],partition=partition)
        scene = folder/'sessions'/experiment.current['id']/'scene.jpg'
        if scene.exists(): shutil.copyfile(scene,folder/'scene.jpg')
    except Exception as error:
        report.update(error=str(error))
        write(folder/'failure-state.json',experiment.state())
    finally:
        if witness_executor:
            witness_executor.shutdown()
            witness_thread.join(timeout=5)
        try:
            experiment.close()
            report['test_simulation_closed']=True
        except Exception as error:
            report.update(passed=False,cleanup_error=str(error))
        finally:
            if witness:
                witness.close(); witness.destroy_node()
            if proxy:
                proxy.destroy_node()
            write(folder/'report.json',report)
    print(json.dumps(report,ensure_ascii=False),flush=True)
    return report['passed']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    parser.add_argument('--trial',choices=('success','fault','natural'),help='One diagnostic trial; default is the fixed five-trial plan')
    args = parser.parse_args()
    os.environ['MOONCAR_EXPERIMENT_ROS_DOMAIN_ID']='90'
    os.environ['ROS_DOMAIN_ID']='90'
    if args.trial:
        return 0 if trial(args.output,args.trial) else 1
    args.output.mkdir(parents=True,exist_ok=False)
    sequence = ['success','fault','success','fault','natural']
    write(args.output/'plan.json',dict(trials=sequence,domain=90,existing_workbench_preserved=True,
          injected_failures_explicit=True,natural_budget_seconds=90,controlled_budget_seconds=120))
    hashes = {str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
              for base in ('scripts','containers','ros2_ws/src') for p in (ROOT/base).rglob('*')
              if p.is_file() and p.suffix in ('.py','.sh','.yaml','.xacro','.sdf') and '__pycache__' not in p.parts}
    write(args.output/'source-sha256.json',hashes)
    rows = []
    for index, kind in enumerate(sequence,1):
        folder = args.output/f'{index:02d}-{kind}'
        print(f'Trial {index}/5: {kind}',flush=True)
        with (args.output/f'{index:02d}-{kind}.log').open('w') as log:
            process = subprocess.run([sys.executable,__file__,str(folder),'--trial',kind],stdout=log,stderr=subprocess.STDOUT)
        path = folder/'report.json'
        row = json.loads(path.read_text()) if path.exists() else dict(passed=False,error='trial process did not produce report')
        row.update(index=index,return_code=process.returncode)
        rows.append(row); write(args.output/'summary.json',dict(complete=len(rows)==len(sequence),
                 passed=len(rows)==len(sequence) and all(r['passed'] and r['return_code']==0 for r in rows),trials=rows))
        print(json.dumps(row,ensure_ascii=False),flush=True)
    return 0 if all(r['passed'] and r['return_code']==0 for r in rows) else 1


if __name__ == '__main__':
    raise SystemExit(main())
