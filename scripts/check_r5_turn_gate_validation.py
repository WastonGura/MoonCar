"""Same fixed contact candidate; route only its test bridge to bounded startup IO."""
import hashlib,json,subprocess,sys
from pathlib import Path

GUARD='/source/ros2_ws/src/mooncar_gazebo/launch/image_dds_guard.py'
TEST_GUARD='/evidence/test-startup/image_dds_guard.py'

def redirect(argv):
    if len(argv)>1 and argv[1]==GUARD:
        manifest=json.loads(Path(TEST_GUARD).with_suffix('.manifest.json').read_text())
        digest=hashlib.sha256(Path(TEST_GUARD).read_bytes()).hexdigest()
        if manifest['test_sha256']!=digest or manifest['production_changed'] or manifest['timeout_changed']:
            raise RuntimeError('Test startup guard signature/configuration mismatch')
        argv=list(argv);argv[1]=TEST_GUARD
    return argv

def main():
    import check_r5_turn_slip_round as original_round
    original=subprocess.Popen
    def launch(argv,*args,**kwargs):return original(redirect(argv),*args,**kwargs)
    subprocess.Popen=launch
    try:code=original_round.main()
    finally:subprocess.Popen=original
    # Keep both errors; a missing grid after startup failure is secondary.
    args=sys.argv
    folder=Path(args[args.index('--output')+1])/args[args.index('--name')+1]
    path=folder/'result.json'
    if path.is_file():
        result=json.loads(path.read_text())
        result['test_startup_manifest']=json.loads(Path(TEST_GUARD).with_suffix('.manifest.json').read_text())
        if result.get('error') and not result.get('schedule_phases'):
            result['secondary_collection_error']=result.get('turn_slip_error')
            result['turn_slip_error']='Primary runtime startup failure: '+result['error']
        path.write_text(json.dumps(result,indent=2)+'\n')
    return code

if __name__=='__main__':raise SystemExit(main())
