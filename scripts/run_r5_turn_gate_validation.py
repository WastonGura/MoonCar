"""Fresh full24 count, unchanged actors; bounded test startup and full tracing."""
import subprocess,sys
from pathlib import Path
import run_r5_turn_slip as base
from run_r5_origin_campaign import sha

MATRIX=base.MATRIX

def main(output):
    base.MATRIX=MATRIX
    original_signature=base.signature
    def signature():
        result=original_signature()
        result['test_startup']={str(p):sha(p) for p in Path('/evidence/test-startup').iterdir() if p.is_file()}
        result['test_gate_shells']={n:sha('/source/scripts/'+n) for n in ('prepare_r5_turn_gate_validation.sh','execute_r5_turn_gate_validation.sh')}
        result['test_gate_tests']={n:sha('/source/ros2_ws/src/mooncar_gazebo/tests/'+n) for n in ('test_r5_startup_guard.py','test_turn_gate_validation.py')}
        return result
    base.signature=signature
    original=subprocess.Popen
    def launch(argv,*args,**kwargs):
        if len(argv)>1 and argv[1]=='/source/scripts/check_r5_turn_slip_round.py':
            argv=list(argv);argv[1]='/source/scripts/check_r5_turn_gate_validation.py'
        return original(argv,*args,**kwargs)
    subprocess.Popen=launch
    try:return base.run(output)
    finally:subprocess.Popen=original;base.signature=original_signature

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('--output',required=True)
    a=p.parse_args();raise SystemExit(main(a.output))
