"""Extra read-only real odometry capture for the origin diagnostic fixture."""
import json
import os
import signal
import subprocess
import threading
import time
from nav_msgs.msg import Odometry
from r5_capture_io import dds_info
from r5_runtime_witness import Witness

class OriginWitness(Witness):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.odom_stopping=False
        self.create_subscription(Odometry,'/odom',self.ros_odom,10)
        self.odom_err=(self.folder/'native-odom.stderr').open('w')
        self.odom_proc=subprocess.Popen(['gz','topic','-e','--json-output','-t','/odom'],
            stdout=subprocess.PIPE,stderr=self.odom_err,text=True,env=args[2],start_new_session=True)
        self.odom_thread=threading.Thread(target=self.native_odom,daemon=True)
        self.odom_thread.start()

    def ros_odom(self,m,info):
        p=m.pose.pose.position;q=m.pose.pose.orientation;v=m.twist.twist
        self.record('ros_wheel_odom',stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
            frame_id=m.header.frame_id,child_frame_id=m.child_frame_id,
            xyz=[p.x,p.y,p.z],quaternion_wxyz=[q.w,q.x,q.y,q.z],
            linear=[v.linear.x,v.linear.y,v.linear.z],angular=[v.angular.x,v.angular.y,v.angular.z],
            pose_covariance=list(m.pose.covariance),twist_covariance=list(m.twist.covariance),**dds_info(info))

    def native_odom(self):
        try:
            for line in self.odom_proc.stdout:
                if not line.strip():continue
                msg=json.loads(line);s=msg.get('header',{}).get('stamp',{})
                self.record('gz_native_odom',stamp=int(s.get('sec',0))+int(s.get('nsec',0))*1e-9,
                            received_wall=time.monotonic(),raw=msg)
            if not self.odom_stopping:self.errors.append('Native odometry observer exited before close')
        except Exception as error:self.errors.append('native odometry: '+repr(error))

    def close(self):
        self.odom_stopping=True
        try:os.killpg(self.odom_proc.pid,signal.SIGINT)
        except ProcessLookupError:pass
        try:self.odom_proc.wait(timeout=3)
        except subprocess.TimeoutExpired:os.killpg(self.odom_proc.pid,signal.SIGKILL);self.odom_proc.wait()
        self.odom_thread.join(timeout=2)
        if self.odom_thread.is_alive():self.errors.append('Native odometry queue not drained')
        self.odom_err.close()
        return super().close()
