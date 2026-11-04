import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_adaptive_turn_node import AdaptiveNode


class AdaptiveNodeTests(unittest.TestCase):
    def setUp(self):
        rclpy.init();self.tmp=tempfile.TemporaryDirectory();self.node=AdaptiveNode(self.tmp.name)
        self.time=0
        self.node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=self.time))
    def tearDown(self):
        self.node.close();self.node.destroy_node();rclpy.shutdown();self.tmp.cleanup()
    def imu(self, stamp, gyro=0.):
        m=Imu();m.header.stamp.nanosec=stamp;m.header.frame_id='imu_link'
        m.orientation.w=1.;m.angular_velocity.z=gyro;self.node.on_imu(m)
    def scan(self,stamp):
        m=LaserScan();m.header.stamp.nanosec=stamp;m.header.frame_id='lidar_scan_frame'
        m.angle_min=-np.pi;m.angle_increment=2*np.pi/1000
        angles=m.angle_min+np.arange(1000)*m.angle_increment
        m.range_min=.05;m.range_max=15.
        m.ranges=np.minimum(3/np.maximum(abs(np.cos(angles)),1e-6),
                           2/np.maximum(abs(np.sin(angles)),1e-6)).tolist()
        return m
    def test_real_ros_callbacks_metadata_and_drain(self):
        self.node.on_request(Twist());self.imu(0);self.node.on_scan(self.scan(0))
        self.time=100000000;self.node.on_request(Twist())
        for t in range(10000000,100000001,10000000):self.imu(t)
        self.node.on_scan(self.scan(100000000))
        self.assertTrue(self.node.close());rows=[json.loads(l) for l in (Path(self.tmp.name)/'adaptive-feedback.jsonl').read_text().splitlines()]
        self.assertEqual(len(rows),2);self.assertEqual([r['event_id'] for r in rows],[1,2])
        self.assertFalse(rows[0]['decision']['valid'])
        self.assertTrue(rows[1]['decision']['valid'],rows[1])
        self.assertEqual(rows[1]['decision']['reason'],'inactive')
        # Close is idempotent and receipts cover every selected metadata row.
        receipts=(Path(self.tmp.name)/'adaptive-writer-receipts.jsonl').read_text().splitlines()
        self.assertEqual(len(receipts),2)
    def test_wrong_frame_never_produces_valid_feedback(self):
        self.node.on_request(Twist());self.imu(0)
        m=self.scan(0);m.header.frame_id='world';self.node.on_scan(m)
        self.assertTrue(self.node.close())
        row=json.loads((Path(self.tmp.name)/'adaptive-feedback.jsonl').read_text())
        self.assertFalse(row['decision']['valid']);self.assertIn('frame',row['decision']['reason'])
    def test_missing_imu_fails_closed(self):
        self.node.on_request(Twist());self.node.on_scan(self.scan(0))
        self.assertTrue(self.node.close())
        row=json.loads((Path(self.tmp.name)/'adaptive-feedback.jsonl').read_text())
        self.assertFalse(row['decision']['valid']);self.assertIn('IMU',row['decision']['reason'])


if __name__=='__main__':unittest.main()
