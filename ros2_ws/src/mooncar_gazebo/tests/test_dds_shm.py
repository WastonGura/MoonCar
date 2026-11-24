import argparse,json,sys,tempfile,unittest,os,subprocess,time,shutil
from pathlib import Path
from types import SimpleNamespace
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_dds_shm import correlation,qualify_candidate,verify_runtime,process_env,runtime_options,fragment_inventory,diagnostic_complete
from run_r5_dds_shm_campaign import progress
from r5_dds_shm import project_runtime
from unittest.mock import patch
from r5_image_delivery import delivery_arguments
from r5_capture_io import image_options,capture_hash,image_argv

class ShmTests(unittest.TestCase):
    def test_missing_ros_overlay_fails_before_any_physics_start(self):
        with patch('ament_index_python.packages.get_package_prefix',side_effect=LookupError('missing')):
            with self.assertRaisesRegex(RuntimeError,'overlay not loaded'):project_runtime()
    def test_control_executables_must_match_current_source(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);installed=root/'install/lib/mooncar_control';installed.mkdir(parents=True)
            source=root/'source/ros2_ws/src/mooncar_control/scripts';source.mkdir(parents=True)
            for name in ('cmd_vel_safety_relay.py','wheel_odometry.py'):
                (installed/name).write_text('print(1)');(installed/name).chmod(0o755);(source/name).write_text('print(1)')
            with patch('ament_index_python.packages.get_package_prefix',return_value=str(root/'install')):
                self.assertEqual(len(project_runtime(root/'source')['executables']),2)
                (installed/'wheel_odometry.py').write_text('print(2)')
                with self.assertRaisesRegex(RuntimeError,'differs'):project_runtime(root/'source')
    def test_stop_preserves_actual_stage_results(self):
        es=[dict(stage='diagnostic',status='passed',passed=True)]*8+[
            dict(stage='calibration',status='passed',passed=True),dict(stage='calibration',status='failed',passed=False)]
        result=progress(es,True,True,'Calibration gap')
        self.assertEqual(result['attempts'],10);self.assertEqual(result['completed_attempts'],10)
        self.assertEqual(result['calibration_rounds'],2);self.assertEqual(result['calibration_passed'],1)
        self.assertTrue(result['repair_verified']);self.assertTrue(result['causal_confirmed'])
    def test_diagnostic_image_gap_does_not_mask_observer_or_input_failure(self):
        r={k:dict(passed=True) for k in ('capture_analysis','image_trace_analysis','delivery_analysis','step_observation','dds')}
        r.update(observer_flush_confirmed=True,formal=dict(command_window=dict(passed=True),contacts_complete=True,
            imagery_observable=True,safe_gap_s=.05,sensors={k:dict(passed=True) for k in ('scan','imu','joints','truth')}))
        self.assertTrue(diagnostic_complete(r,True))
        r['formal']['command_window']['passed']=False
        with self.assertRaises(RuntimeError):diagnostic_complete(r,True)
        r['formal']['command_window']['passed']=True;r['dds']['passed']=False
        with self.assertRaises(RuntimeError):diagnostic_complete(r,True)
    def rows(self):
        base=dict(segment='aabb',offset=123,validity=8,writer='writer',sequence=42,
            fragment=3,stamp_ns=None,wall_ns=2,pid=10,a=1,b=0,c=1,d=1)
        return [dict(base,stage='fragment_tx',stamp_ns=14100000000),
            dict(base,stage='capacity_recovery',wall_ns=3),dict(base,stage='buffer_invalid',pid=20,wall_ns=4),
            dict(base,stage='reassembly_replaced',pid=20,wall_ns=5)]
    def test_exact_tuple_and_sequence_link(self):
        links=correlation(self.rows(),{14100000000});self.assertEqual(len(links),1)
        self.assertTrue(links[0]['incomplete_replaced'])
    def test_equal_rgb_depth_timestamps_keep_writer_stream_identity(self):
        kinds={'writer':'rgb'}
        self.assertEqual(len(correlation(self.rows(),{('rgb',14100000000)},kinds)),1)
        self.assertFalse(correlation(self.rows(),{('depth',14100000000)},kinds))
        self.assertFalse(correlation(self.rows(),{('rgb',14100000000)},{}))
    def test_normal_release_is_not_forced_recovery(self):
        rows=self.rows();rows[1]['stage']='normal_release';self.assertFalse(correlation(rows,{14100000000}))
    def test_no_waiting_references_not_causal(self):
        rows=self.rows();rows[1]['a']=0;self.assertFalse(correlation(rows,{14100000000}))
    def test_failed_recovery_not_causal(self):
        rows=self.rows();rows[1]['c']=0;self.assertFalse(correlation(rows,{14100000000}))
    def test_old_validity_does_not_match_new_buffer(self):
        rows=self.rows();rows[2]['validity']=9;self.assertFalse(correlation(rows,{14100000000}))
    def test_offset_and_segment_must_match(self):
        for field,value in [('segment','other'),('offset',124)]:
            rows=self.rows();rows[2][field]=value;self.assertFalse(correlation(rows,{14100000000}))
    def test_missing_sequence_stamp_not_fabricated(self):
        rows=self.rows();rows[0]['stamp_ns']=None;self.assertFalse(correlation(rows,{14100000000}))
    def test_port_drop_does_not_qualify_size_fix(self):
        rows=self.rows();rows[1]['stage']='port_discard';self.assertFalse(correlation(rows,{14100000000}))
    def test_received_frame_not_classified_as_loss(self):self.assertFalse(correlation(self.rows(),set()))
    def test_both_observed_rounds_required(self):
        es=[dict(passed=False)]*4
        with self.assertRaises(RuntimeError):qualify_candidate(es,[dict(dds=dict(passed=True,capacity_qualified=True)),dict(dds=dict(passed=True))])
        self.assertEqual(qualify_candidate(es,[dict(dds=dict(passed=True,capacity_qualified=True))]*2),'shm_tx8m')
    def test_unreproduced_stops_without_calibration(self):
        with self.assertRaises(RuntimeError):qualify_candidate([dict(passed=True)]*4,[])
    def test_observer_interference_stops(self):
        with self.assertRaises(RuntimeError):qualify_candidate([dict(passed=True)]*2+[dict(passed=False)]*2,[])
    def test_stock_cannot_claim_trace_or_capacity_fix(self):
        for args in [SimpleNamespace(dds_runtime='stock',dds_trace='images'),
                     SimpleNamespace(dds_runtime='stock',dds_trace='off',image_delivery_profile='shm_tx8m')]:
            with self.assertRaises(RuntimeError):verify_runtime(args)
    def test_missing_library_manifest_fails_closed(self):
        with tempfile.TemporaryDirectory() as p:
            with self.assertRaises(FileNotFoundError):verify_runtime(SimpleNamespace(dds_runtime='observed',dds_prefix=p))
    def test_only_bridge_segment_changes(self):
        args=SimpleNamespace(dds_prefix='/work/observed',dds_runtime='observed',dds_trace='images',image_delivery_profile='shm_tx8m')
        for label in ('main','bridge','capture-probe','cpp-images','safety-relay'):
            env=process_env(args,{'LD_LIBRARY_PATH':'/opt/ros/jazzy/lib'},'/evidence/round',label)
            self.assertEqual('MOONCAR_DDS_SHM_TX_BYTES' in env,label=='bridge')
            self.assertEqual('MOONCAR_DDS_TRACE' in env,label!='safety-relay')
    def test_default_and_flag_propagation(self):
        p=argparse.ArgumentParser();delivery_arguments(p);args=p.parse_args([])
        self.assertEqual(runtime_options(args),{})
        self.assertEqual(capture_hash('async',**image_options(args)),'0429e34e9d0afa513ac7af279f6316c7613f9a06dee0a135b13fe53b6583312f')
        self.assertIn('--dds-runtime',image_argv(args))
    def test_fragment_missing_duplicate_and_out_of_order(self):
        base=dict(stage='fragment_rx',pid=1,writer='w',reader='r',sequence=2,a=4,b=1,fragments=1)
        out=fragment_inventory([dict(base,fragment=n) for n in (1,3,3,2)])[0]
        self.assertEqual(out['missing_fragments'],[4]);self.assertEqual(out['duplicate_fragments'],[3]);self.assertTrue(out['out_of_order'])
    def test_fragment_sequences_never_merge(self):
        base=dict(stage='fragment_rx',pid=1,writer='w',reader='r',a=2,b=1,fragments=1,fragment=1)
        self.assertEqual(len(fragment_inventory([dict(base,sequence=n) for n in (1,2)])),2)
    def test_missing_fragment_identity_stays_null(self):
        base=dict(stage='fragment_rx',pid=1,writer='w',reader='r',a=2,b=1,fragments=1,fragment=1,sequence=None)
        out=fragment_inventory([base])[0];self.assertIsNone(out['sequence']);self.assertIsNone(out['missing_fragments'])
    def test_failed_output_and_unflushed_writer_are_distinct(self):
        from r5_image_delivery import writer_complete
        self.assertFalse(writer_complete([],dict(drained=True,failed=True,received=0,written=0)))
        self.assertFalse(writer_complete([],dict(drained=False,failed=False,received=0,written=0)))
    def test_wrong_version_or_library_hash_cannot_fall_back(self):
        import hashlib
        with tempfile.TemporaryDirectory() as p:
            root=Path(p);lib=root/'test.so';lib.write_bytes(b'initial')
            manifest=dict(versions=dict(fastdds='2.14.7',rmw_fastrtps_shared_cpp='8.4.4',rmw_fastrtps_cpp='8.4.4'),
                libraries={'test.so':hashlib.sha256(lib.read_bytes()).hexdigest()})
            args=SimpleNamespace(dds_runtime='observed',dds_prefix=p)
            (root/'manifest.json').write_text(json.dumps(manifest))
            with self.assertRaises(RuntimeError):verify_runtime(args)
            manifest['versions']['fastdds']='2.14.6';(root/'manifest.json').write_text(json.dumps(manifest))
            self.assertEqual(verify_runtime(args),manifest)
            lib.write_bytes(b'changed')
            with self.assertRaises(RuntimeError):verify_runtime(args)
    def test_size_candidate_requires_matching_complete_evidence(self):
        with tempfile.TemporaryDirectory() as p:
            root=Path(p);(root/'manifest.json').write_text(json.dumps(dict(versions=dict(fastdds='2.14.6',
                rmw_fastrtps_shared_cpp='8.4.4',rmw_fastrtps_cpp='8.4.4'),libraries={})))
            with self.assertRaises(RuntimeError):verify_runtime(SimpleNamespace(dds_runtime='observed',dds_prefix=p,
                image_delivery_profile='shm_tx8m'))

class BoundedWriterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not shutil.which('g++'):raise unittest.SkipTest('Native logger regression requires g++')
        cls.temp=tempfile.TemporaryDirectory();cls.root=Path(cls.temp.name)
        source=Path(__file__).resolve().parents[4]/'containers/r5_dds_observer'
        cls.library=cls.root/'libtrace.so';cls.fixture=cls.root/'fixture'
        subprocess.run(['g++','-std=c++17','-O2','-shared','-fPIC',str(source/'Trace.cc'),'-pthread','-o',str(cls.library)],check=True)
        subprocess.run(['g++','-std=c++17',str(source/'WriterFixture.cc'),'-ldl','-o',str(cls.fixture)],check=True)
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def exercise(self,name,count,delayed=False,missing=False):
        folder=self.root/name
        if not delayed and not missing:folder.mkdir()
        prefix=folder/'trace';env=dict(os.environ,MOONCAR_DDS_TRACE=str(prefix))
        child=subprocess.Popen([str(self.fixture),str(self.library),str(count)],env=env)
        if delayed:time.sleep(.15);folder.mkdir()
        self.assertEqual(child.wait(15),0)
        logfile=Path(str(prefix)+f'.{child.pid}.jsonl');status=Path(str(logfile)+'.status.json')
        return logfile,status
    def test_library_loaded_before_directory_creation_drains(self):
        path,status=self.exercise('late-directory',100,delayed=True)
        s=json.loads(status.read_text());self.assertTrue(s['drained']);self.assertFalse(s['failed'])
        self.assertEqual(s['written'],100);self.assertEqual(len(path.read_text().splitlines()),100)
        self.assertIsNone(json.loads(path.read_text().splitlines()[0])['writer'])
    def test_archive_worker_without_dds_has_complete_empty_log(self):
        path,status=self.exercise('archive-empty',0)
        s=json.loads(status.read_text());self.assertTrue(s['drained']);self.assertFalse(s['failed'])
        self.assertEqual(s['submitted'],0);self.assertEqual(path.read_text(),'')
    def test_preallocated_queue_overflow_fails_without_silent_drop(self):
        path,status=self.exercise('overflow',1000000)
        s=json.loads(status.read_text());self.assertFalse(s['drained']);self.assertTrue(s['failed']);self.assertGreater(s['overflow'],0)
        self.assertLessEqual(s['high_water'],8192);self.assertEqual(s['written']+s['overflow'],s['submitted'])
    def test_unavailable_output_directory_never_reports_success(self):
        path,status=self.exercise('missing-directory',10,missing=True)
        self.assertFalse(path.exists());self.assertFalse(status.exists())
    def test_actual_disk_write_failure_is_registered(self):
        if not Path('/dev/full').exists():self.skipTest('Linux write-failure device unavailable')
        folder=self.root/'disk-full';folder.mkdir();prefix=folder/'trace'
        code='import os,sys; os.symlink("/dev/full",sys.argv[1]+"."+str(os.getpid())+".jsonl"); os.execl(sys.argv[2],sys.argv[2],sys.argv[3],"1000")'
        child=subprocess.Popen([sys.executable,'-c',code,str(prefix),str(self.fixture),str(self.library)],
            env=dict(os.environ,MOONCAR_DDS_TRACE=str(prefix)))
        self.assertEqual(child.wait(15),0)
        status=json.loads(Path(str(prefix)+f'.{child.pid}.jsonl.status.json').read_text())
        self.assertTrue(status['failed']);self.assertFalse(status['drained'])

if __name__=='__main__':unittest.main()
