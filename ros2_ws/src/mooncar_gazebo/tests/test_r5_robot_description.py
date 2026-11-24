"""Support-description binding and real TF feedback, with no physics server."""
import copy
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import unittest
from argparse import Namespace
import xml.etree.ElementTree as ET
from r5_robot_description import description_path,description_parameters,validate_description,validate_bound_description,qualify_feedback

class DescriptionTests(unittest.TestCase):
    def manifest(self):return json.loads(Path(os.environ.get('R5_SUSPENSION_PREPARED','/diagnosis/prepared-fixed')+'/manifest.json').read_text())

    def test_explicit_candidate_is_not_overridden_by_archived_fixture_model(self):
        args=Namespace(model='/oldclosure/prepared-d0/formal.sdf',robot_description_urdf='/diagnosis/after/suspended.urdf')
        self.assertEqual(str(description_path(args)),args.robot_description_urdf)
        self.assertEqual(str(description_path(Namespace(model=args.model))),'/oldclosure/prepared-d0/formal.urdf')

    def test_research_tf_rate_is_explicit_and_legacy_default_is_preserved(self):
        urdf=str(Path(__file__).resolve().parent/'fixtures/r5-d0.urdf')
        args=Namespace(model='/oldclosure/prepared-d0/formal.sdf',robot_description_urdf=urdf)
        self.assertNotIn('publish_frequency',description_parameters(args))
        args.robot_state_publish_frequency_hz=30.
        self.assertEqual(description_parameters(args)['publish_frequency'],30.)
        args.robot_state_publish_frequency_hz=float('nan')
        with self.assertRaises(ValueError):description_parameters(args)

    @unittest.skipUnless(Path('/diagnosis/after/rigid.urdf').is_file(),'Isolated /diagnosis URDF fixture required')
    def test_actual_tf_cadence_reproduces_20hz_gap_and_30hz_fixes_it(self):
        import yaml
        import rclpy
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        from rosgraph_msgs.msg import Clock
        urdf=Path('/diagnosis/after/rigid.urdf')
        names=[j.get('name') for j in ET.parse(urdf).findall('joint') if j.get('type')!='fixed']
        results=[]
        for frequency in (None,30.):
            with tempfile.TemporaryDirectory() as temporary:
                folder=Path(temporary);config=folder/'robot.yaml'
                parameters=description_parameters(Namespace(model='unused',robot_description_urdf=urdf,robot_state_publish_frequency_hz=frequency))
                config.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':parameters}}))
                with (folder/'process.log').open('w') as log:
                    process=subprocess.Popen(['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(config)],stdout=log,stderr=log,start_new_session=True)
                    rclpy.init();node=rclpy.create_node('r5_tf_cadence_regression');rows=[]
                    publisher=node.create_publisher(JointState,'/joint_states',100)
                    clock=node.create_publisher(Clock,'/clock',10)
                    def receive(message):
                        rows.append(dict(kind='robot_tf',transforms=[dict(stamp_ns=t.header.stamp.sec*1_000_000_000+t.header.stamp.nanosec,
                            parent=t.header.frame_id,child=t.child_frame_id,
                            translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                            rotation_xyzw=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]) for t in message.transforms]))
                    node.create_subscription(TFMessage,'/tf',receive,100)
                    try:
                        deadline=time.monotonic()+5
                        while (publisher.get_subscription_count()==0 or clock.get_subscription_count()==0) and time.monotonic()<deadline:
                            rclpy.spin_once(node,timeout_sec=.01)
                        self.assertGreater(publisher.get_subscription_count(),0);self.assertGreater(clock.get_subscription_count(),0)
                        # Establish actual TF reception before the measured
                        # trace, just as the physical fixture warms up first.
                        stamp=0;ready_until=time.monotonic()+3
                        while not rows and time.monotonic()<ready_until:
                            stamp+=100_000_000
                            tick=Clock();tick.clock.sec=stamp//1_000_000_000;tick.clock.nanosec=stamp%1_000_000_000;clock.publish(tick)
                            until=time.monotonic()+.03
                            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
                            prime=JointState();prime.header.stamp=tick.clock;prime.name=names;prime.position=[0.]*len(names);publisher.publish(prime)
                            until=time.monotonic()+.03
                            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
                        self.assertTrue(rows,'TF discovery warmup did not complete');rows.clear()
                        for i in range(32):
                            stamp+=33_500_000 if i%2==0 else 67_000_000
                            tick=Clock();tick.clock.sec=stamp//1_000_000_000;tick.clock.nanosec=stamp%1_000_000_000
                            clock.publish(tick)
                            until=time.monotonic()+.02
                            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
                            msg=JointState();msg.header.stamp=tick.clock;msg.name=names;msg.position=[0.]*len(names)
                            publisher.publish(msg);rows.append(dict(kind='joints',stamp=stamp/1e9,stamp_ns=stamp,names=names,positions=list(msg.position)))
                            until=time.monotonic()+.03
                            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.002)
                        (folder/'observations.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
                        verdict=qualify_feedback(folder,urdf);results.append(dict(explicit_frequency_hz=frequency,**verdict))
                    finally:
                        os.killpg(process.pid,signal.SIGINT);process.wait(timeout=5);node.destroy_node();rclpy.shutdown()
        with Path(os.environ.get('R5_TF_CADENCE_EVIDENCE','/diagnosis/tf-cadence-regression.json')).open('x') as stream:
            stream.write(json.dumps(results,indent=2)+'\n')
        self.assertFalse(results[0]['passed'],results)
        self.assertAlmostEqual(results[0]['maximum_matched_gap_s'],.1005)
        self.assertTrue(results[1]['passed'],results)
        self.assertEqual(results[1]['matched_joint_tf_samples'],32)

    @unittest.skipUnless(Path(os.environ.get('R5_SUSPENSION_PREPARED','/diagnosis/prepared-fixed')+'/manifest.json').is_file(),'Isolated /diagnosis prepared manifest required')
    def test_rigid_description_rejected_for_suspension_before_simulation(self):
        manifest=self.manifest();spec=manifest['rounds'][2]
        self.assertTrue(validate_bound_description(spec)['passed'])
        with self.assertRaisesRegex(ValueError,'dynamic joints'):
            validate_description(spec['world'],'/oldclosure/prepared-d0/formal.urdf')
        invalid=copy.deepcopy(spec);invalid['robot_description_sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'hash'):validate_bound_description(invalid)

    @unittest.skipUnless(Path('/diagnosis/after/suspended.urdf').is_file(),'Isolated /diagnosis URDF fixture required')
    def test_missing_tf_feedback_is_not_compatibility_success(self):
        with tempfile.TemporaryDirectory() as folder:
            (Path(folder)/'observations.jsonl').write_text('')
            result=qualify_feedback(folder,'/diagnosis/after/suspended.urdf')
            self.assertFalse(result['passed']);self.assertEqual(result['matched_joint_tf_samples'],0)

    @unittest.skipUnless(Path('/diagnosis/after/suspended.urdf').is_file(),'Isolated /diagnosis URDF fixture required')
    def test_actual_robot_state_publisher_tracks_passive_suspension(self):
        import yaml
        import rclpy
        from sensor_msgs.msg import JointState
        from tf2_msgs.msg import TFMessage
        urdf=Path('/diagnosis/after/suspended.urdf')
        description=ET.parse(urdf)
        expected={j.get('name'):.0123 if j.get('type')=='prismatic' else 0.03 for j in description.findall('joint') if j.get('type')!='fixed'}
        with tempfile.TemporaryDirectory() as folder:
            folder=Path(folder);config=folder/'robot.yaml'
            config.write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'robot_description':urdf.read_text(),'use_sim_time':False}}}))
            with (folder/'process.log').open('w') as log:
                process=subprocess.Popen(['ros2','run','robot_state_publisher','robot_state_publisher','--ros-args','--params-file',str(config)],stdout=log,stderr=log,start_new_session=True)
                rows=[];rclpy.init();node=rclpy.create_node('r5_description_feedback_regression')
                publisher=node.create_publisher(JointState,'/joint_states',100)
                def receive(message):
                    rows.append(dict(kind='robot_tf',transforms=[dict(stamp_ns=t.header.stamp.sec*1_000_000_000+t.header.stamp.nanosec,
                        parent=t.header.frame_id,child=t.child_frame_id,
                        translation=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z],
                        rotation_xyzw=[t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w]) for t in message.transforms]))
                node.create_subscription(TFMessage,'/tf',receive,100)
                try:
                    deadline=time.monotonic()+5
                    while publisher.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.01)
                    self.assertGreater(publisher.get_subscription_count(),0)
                    for i in range(24):
                        msg=JointState();msg.header.stamp=node.get_clock().now().to_msg();msg.name=list(expected);msg.position=list(expected.values())
                        publisher.publish(msg);rows.append(dict(kind='joints',stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                            stamp_ns=msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec,names=list(msg.name),positions=list(msg.position)))
                        until=time.monotonic()+.06
                        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.005)
                    (folder/'observations.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
                    result=qualify_feedback(folder,urdf);self.assertTrue(result['passed'],result)
                    self.assertEqual(result['dynamic_joints'],10)
                finally:
                    os.killpg(process.pid,signal.SIGINT);process.wait(timeout=5)
                    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
