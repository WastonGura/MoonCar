"""Test-only fixed candidate matrix and strict, actual-step qualification."""
import math

MATRIX=tuple(("flat",step,kind+f"-{repeat}",linear,angular)
    for step in (.001,.0005)
    for kind,linear,angular in (("straight",.1,0.),("left",0.,.2),("right",0.,-.2))
    for repeat in (1,2))
MATRIX=MATRIX[:6]+(("flat",.0005,"zero",0.,0.),)+MATRIX[6:]+tuple(
    (scene,.001,kind,linear,angular) for scene in ("ramp","curve","survey")
    for kind,linear,angular in (("zero",0.,0.),("straight",.1,0.),("left",0.,.2),("right",0.,-.2)))

def validate(scene,step):
    if scene not in ("flat","ramp","curve","survey") or not math.isfinite(step) or step not in (.001,.0005,.00025):
        raise ValueError('Outside frozen scene/step matrix')

def qualify_grid(previous,schedule):
    """Retain all old non-time gates; replace only its hard-coded 20000 steps."""
    step=schedule['step'];validate('flat',step)
    ns=round(step*1e9);motion=schedule['motion'];before=motion['before'];after=motion['after']
    count=after['iteration']-before['iteration']
    elapsed=round(after['sim']*1e9)-round(before['sim']*1e9)
    expected=20_000_000_000//ns
    exact=bool(schedule['mode']=='paused_steps' and count==expected and
        abs(elapsed-20_000_000_000)<=ns and abs(elapsed-count*ns)<=1)
    complete=bool(previous['world_pose_complete'] and previous['per_step_grid'] and previous['frames_valid'] and
        previous['odometry'] and all(x['passed'] for x in previous['odometry'].values()) and
        previous['contacts'] and all(x['complete'] for x in previous['contacts'].values()))
    return dict(previous,passed=complete and exact,actual_step_grid_passed=exact,
        expected_motion_steps=expected,actual_step_s=step,legacy_fixed_1ms_gate_passed=previous['passed'])
