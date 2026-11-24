import os
import copy
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import numpy as np
from analyze_r5_static_diagnosis import relative_z
from prepare_r5_static_diagnosis import bench_world
from r5_trial_processes import cleanup,owned
from check_r5_suspension_round import qualify_observation
from r5_suspension_protocol import validate_campaign,validate_environment
from r5_suspension_comparison import first_crossings
from unittest.mock import patch
from types import SimpleNamespace


class MechanicsDiagnosticTests(unittest.TestCase):
    def test_already_stopped_transport_still_flushes_and_closes_files(self):
        import io
        from r5_trial_processes import finish_transport_stream
        raw=io.StringIO();err=io.StringIO()
        child=SimpleNamespace(pid=123,poll=lambda:None,wait=lambda **kwargs:0)
        with patch('r5_trial_processes.os.killpg',side_effect=ProcessLookupError):
            finish_transport_stream(child,raw,err)
        self.assertTrue(raw.closed and err.closed)

    def test_abort_pause_uses_owned_server_partition_not_parent_partition(self):
        from r5_trial_processes import server_context
        rows=[dict(pid=10,start_ticks=100,state='S',command='gz sim -s own.sdf',gz_partition='actual-server-partition')]
        self.assertEqual(server_context(rows)['GZ_PARTITION'],'actual-server-partition')
        calls=[]
        def request(argv,**kwargs):
            calls.append(kwargs['env']['GZ_PARTITION'])
            return SimpleNamespace(returncode=0,stdout='data: true\n' if argv[1]=='service' else '',stderr='')
        with patch('r5_trial_processes.owned',side_effect=[rows,[],[]]),patch('r5_trial_processes.subprocess.run',side_effect=request),patch('r5_trial_processes.time.sleep'):
            result=cleanup('own','world',True)
        self.assertTrue(result['passed']);self.assertEqual(calls,['actual-server-partition']*2)

    def test_pause_timeout_with_exit_zero_is_cleanup_failure(self):
        rows=[dict(pid=10,start_ticks=100,state='S',command='gz sim -s own.sdf',gz_partition='actual-server-partition')]
        for response in ('Service call timed out\n','data: false\n',''):
            with self.subTest(response=response),patch('r5_trial_processes.owned',side_effect=[rows,[],[]]),patch('r5_trial_processes.subprocess.run',return_value=SimpleNamespace(returncode=0,stdout=response,stderr='')),patch('r5_trial_processes.time.sleep'):
                result=cleanup('own','world',True)
                self.assertFalse(result['passed']);self.assertFalse(result['stop_pause_passed']);self.assertEqual(result['owned_after'],[])

    def test_missing_localhost_discovery_rejected_before_launch(self):
        with self.assertRaisesRegex(ValueError,'LOCALHOST'):
            validate_environment(dict(ROS_DOMAIN_ID='95',ROS_AUTOMATIC_DISCOVERY_RANGE='',GZ_PARTITION='isolated'))
    def test_same_spring_reference_with_interior_load_and_both_limit_directions(self):
        p=dict(force_N=2.924514673566879,k_N_m=292.4514673566879,
               c_N_s_m=32.16802022557981,reference_m=-.01,lower_m=-.02,upper_m=.02)
        for case,expected in [('load',.01),('decay',-.01),('lower',-.02),('upper',.02)]:
            tree,proof=bench_world(p,case,'OUT')
            self.assertAlmostEqual(proof['expected_q'],expected)
            self.assertEqual(float(tree.findtext('.//spring_reference')),-.01)
            self.assertEqual(float(tree.findtext('.//spring_stiffness')),p['k_N_m'])

    def test_link_displacement_uses_rotated_parent_axis(self):
        data={f'{part}_{axis}':np.array([0.]) for part in ('parent','child') for axis in ('x','y','z','qw','qx','qy','qz')}
        data['parent_qw']=np.array([2**-.5]);data['parent_qy']=np.array([2**-.5])
        data['child_x']=np.array([.003])
        self.assertAlmostEqual(relative_z(data,'')[0],.003)

    def test_missing_step_and_nonfinite_measured_state_rejected(self):
        data=np.array([(1,.001,0.,np.nan),(2,.002,0.,np.nan)],
                      dtype=[('iteration',float),('t',float),('spring_q',float),('force_cmd',float)])
        self.assertTrue(qualify_observation(data,data))
        self.assertFalse(qualify_observation(data[:1],data))
        invalid=data.copy();invalid['spring_q'][1]=np.nan
        self.assertFalse(qualify_observation(invalid,data))

    def test_common_angle_uses_first_crossing_not_a_later_return(self):
        theta=np.array([0.,1.,.8,2.]);points=np.array([[0.],[1.],[100.],[2.]])
        self.assertTrue(np.allclose(first_crossings(theta,points,np.array([.5,1.,1.5])),[[.5],[1.],[1.5]]))

    @unittest.skipUnless(Path('/proc').is_dir(),'Linux process ownership regression')
    def test_orphan_cleanup_preserves_unrelated_process(self):
        token='regression-static-owned-'+str(os.getpid())
        tagged=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'],env=dict(os.environ,MOONCAR_TRIAL_ID=token))
        unrelated=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)'])
        try:
            deadline=time.monotonic()+2
            while not owned(token) and time.monotonic()<deadline:time.sleep(.01)
            identities=owned(token);self.assertEqual([r['pid'] for r in identities],[tagged.pid])
            self.assertGreater(identities[0]['start_ticks'],0)
            result=cleanup(token,'unused',False)
            self.assertTrue(result['passed']);self.assertEqual(result['owned_after'],[])
            tagged.wait(timeout=2);self.assertIsNone(unrelated.poll())
        finally:
            for process in (tagged,unrelated):
                if process.poll() is None:process.terminate()
                process.wait(timeout=2)

    @unittest.skipUnless(Path('/proc').is_dir(),'Linux identity/zombie regression')
    def test_known_zombie_is_a_residual_even_after_environment_disappears(self):
        code='import os,time; p=os.fork(); (time.sleep(.1),os._exit(0)) if p==0 else (print(p,flush=True),time.sleep(60))'
        parent=subprocess.Popen([sys.executable,'-c',code],stdout=subprocess.PIPE,text=True)
        try:
            pid=int(parent.stdout.readline());path=Path(f'/proc/{pid}/stat')
            stat=path.read_text().rsplit(')',1)[1].split();known=[dict(pid=pid,start_ticks=int(stat[19]),command='regression child')]
            deadline=time.monotonic()+2
            while path.exists() and path.read_text().rsplit(')',1)[1].split()[0]!='Z' and time.monotonic()<deadline:time.sleep(.01)
            residual=owned('deliberately-no-live-environment',known)
            self.assertEqual([(r['pid'],r['state']) for r in residual],[(pid,'Z')])
            stale=[dict(known[0],start_ticks=known[0]['start_ticks']+1)]
            self.assertEqual(owned('deliberately-no-live-environment',stale),[])
        finally:
            parent.terminate();parent.wait(timeout=2);parent.stdout.close()


@unittest.skipUnless(Path(os.environ.get('R5_SUSPENSION_PREPARED','/diagnosis/prepared')+'/manifest.json').exists(),'Isolated 17-round preparation required')
class FrozenProtocolTests(unittest.TestCase):
    def setUp(self):self.manifest=json.loads(Path(os.environ.get('R5_SUSPENSION_PREPARED','/diagnosis/prepared')+'/manifest.json').read_text())

    def test_exact_seventeen_rounds_and_order_required(self):
        invalid=copy.deepcopy(self.manifest);invalid['rounds'].pop()
        with self.assertRaisesRegex(ValueError,'seventeen'):validate_campaign(invalid)
        invalid=copy.deepcopy(self.manifest);invalid['rounds'][0],invalid['rounds'][1]=invalid['rounds'][1],invalid['rounds'][0]
        with self.assertRaisesRegex(ValueError,'order'):validate_campaign(invalid)

    def test_wrong_world_and_wrong_imported_parameter_rejected(self):
        invalid=copy.deepcopy(self.manifest);invalid['rounds'][0]['world_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'world hash'):validate_campaign(invalid)
        with tempfile.TemporaryDirectory() as folder:
            invalid=copy.deepcopy(self.manifest);spec=invalid['rounds'][2];tree=ET.parse(spec['world'])
            tree.find(".//joint[@name='front_left_wheel_suspension_joint']/axis/dynamics/spring_stiffness").text='123'
            path=Path(folder)/'world.sdf';tree.write(path);spec['world']=str(path)
            spec['world_sha256']=hashlib.sha256(path.read_bytes()).hexdigest()
            with self.assertRaisesRegex(ValueError,'parameter'):validate_campaign(invalid)

if __name__=='__main__':unittest.main()
