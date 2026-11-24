"""No simulator or physics steps: witness close, faults and shared deadline."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from r5_capture_io import EventWriter
from r5_probe_shutdown import shutdown_witness

class Child:
    def __init__(self,delay=0):self.delay=delay;self.stopped=False;self.signals=0
    def poll(self):return 0 if self.stopped else None
    def send_signal(self,value):self.signals+=1
    def wait(self,timeout):
        if self.delay>timeout:
            time.sleep(timeout);raise subprocess.TimeoutExpired('mock reader',timeout)
        time.sleep(self.delay);self.stopped=True;return 0
    def kill(self):self.stopped=True

class ProbeShutdownTests(unittest.TestCase):
    def test_parallel_reader_and_file_close_under_original_five_seconds(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors,test_close_delay=2.8)
            for i in range(100):writer.submit(dict(event_id=i))
            reader=threading.Thread(target=lambda:time.sleep(2.8));reader.start()
            started=time.monotonic()
            result=shutdown_witness(Child(2.8),reader,writer,errors,folder)
            self.assertTrue(result['passed']);self.assertLess(time.monotonic()-started,5)
            self.assertEqual(result['events']['written'],100)
            self.assertTrue(result['events']['closed'] and result['events']['ids_verified'])

    def test_normal_and_repeat_close(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors)
            writer.submit(dict(event_id=1));first=writer.finish(time.monotonic()+1)
            self.assertEqual(first,writer.finish(time.monotonic()+1));self.assertTrue(first['drained'])
            with self.assertRaises(RuntimeError):writer.submit(dict(event_id=2))

    def test_missing_receipt_and_duplicate_ids_rejected(self):
        for mode in ('missing','duplicate'):
            with self.subTest(mode=mode),tempfile.TemporaryDirectory() as folder:
                errors=[];writer=EventWriter(folder,errors,test_close_delay=.05)
                writer.submit(dict(event_id=1))
                if mode=='duplicate':writer.submit(dict(event_id=1))
                writer.begin_finish(time.monotonic()+1)
                if mode=='missing':
                    deadline=time.monotonic()+1
                    while writer.written<1 and time.monotonic()<deadline:time.sleep(.001)
                    (Path(folder)/'writer-receipts.jsonl').unlink()
                self.assertFalse(writer.finish(time.monotonic()+1)['drained']);self.assertTrue(errors)

    def test_write_exception_is_failure(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors);writer.submit(dict(event_id=1,value=float('nan')))
            self.assertFalse(writer.finish(time.monotonic()+1)['drained']);self.assertTrue(errors)

    def test_timeout_retains_partial_files_stacks_and_failure_latches(self):
        with tempfile.TemporaryDirectory() as folder:
            errors=[];writer=EventWriter(folder,errors,test_close_delay=.2)
            writer.submit(dict(event_id=1));reader=threading.Thread(target=lambda:time.sleep(.2));reader.start()
            result=shutdown_witness(Child(.2),reader,writer,errors,folder,seconds=.03)
            self.assertFalse(result['passed']);self.assertTrue((Path(folder)/'probe-shutdown-stacks.json').exists())
            self.assertTrue((Path(folder)/'observations.jsonl').exists())
            reader.join();writer.thread.join();self.assertFalse(writer.finish(time.monotonic()+1)['drained'])

    @unittest.skipIf(sys.platform=='win32','POSIX SIGINT integration runs in dedicated container')
    def test_real_sigint_reader_exit(self):
        with tempfile.TemporaryDirectory() as folder:
            child=subprocess.Popen([sys.executable,'-u','-c',
                'import signal,time; signal.signal(signal.SIGINT,lambda *a:exit(0)); print("ready",flush=True); time.sleep(30)'],stdout=subprocess.PIPE,text=True)
            self.assertEqual(child.stdout.readline().strip(),'ready')
            reader=threading.Thread(target=lambda:list(child.stdout));reader.start()
            errors=[];writer=EventWriter(folder,errors);writer.submit(dict(event_id=1))
            self.assertTrue(shutdown_witness(child,reader,writer,errors,folder)['passed']);child.stdout.close()

    @unittest.skipIf(sys.platform=='win32','ROS witness integration runs in dedicated container')
    def test_real_ros_witness_sigint_without_any_simulator(self):
        import os
        import rclpy
        from sensor_msgs.msg import Imu
        from rclpy.qos import qos_profile_sensor_data
        saved_env={k:os.environ.get(k) for k in ('ROS_AUTOMATIC_DISCOVERY_RANGE','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY')}
        os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
        os.environ['ROS_DOMAIN_ID']='92'
        os.environ.pop('ROS_LOCALHOST_ONLY',None)
        try:
            with tempfile.TemporaryDirectory() as folder:
                with (Path(folder)/'process.log').open('w') as log:
                    process=subprocess.Popen([sys.executable,'-B',str(Path(__file__).resolve().parents[4]/'scripts/r5_capture_probe.py'),
                        '--output',folder,'--image-trace','headers'],stdout=log,stderr=log)
                    rclpy.init();node=rclpy.create_node('witness_shutdown_regression')
                    publisher=node.create_publisher(Imu,'/mooncar/imu',qos_profile_sensor_data)
                    try:
                        deadline=time.monotonic()+5
                        while publisher.get_subscription_count()==0 and time.monotonic()<deadline:
                            rclpy.spin_once(node,timeout_sec=.01)
                        self.assertGreater(publisher.get_subscription_count(),0)
                        for i in range(1200):
                            message=Imu();message.header.stamp.nanosec=i
                            publisher.publish(message);time.sleep(.001)
                        time.sleep(.1);started=time.monotonic()
                        import signal
                        process.send_signal(signal.SIGINT);process.wait(timeout=8)
                        self.assertLess(time.monotonic()-started,5)
                        self.assertEqual(process.returncode,0,(Path(folder)/'process.log').read_text())
                        status=json.loads((Path(folder)/'probe-status.json').read_text())
                        self.assertTrue(status['passed'],status);self.assertGreater(status['events']['written'],500)
                        self.assertEqual(status['events']['submitted'],status['events']['written'])
                    finally:
                        if process.poll() is None:process.kill();process.wait()
                        node.destroy_node();rclpy.shutdown()
        finally:
            for k,v in saved_env.items():
                if v is None:os.environ.pop(k,None)
                else:os.environ[k]=v

if __name__=='__main__':unittest.main()
