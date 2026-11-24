"""Robot description adapter; the legacy output is byte-for-byte xacro output."""
import argparse
import subprocess
from turn_guard import augment_urdf, verify

def main():
    p=argparse.ArgumentParser()
    p.add_argument('--model', required=True)
    p.add_argument('--profile', choices=('legacy','r5_v2','r5_support_v1'), required=True)
    p.add_argument('--root', required=True)
    p.add_argument('--use-sim-time', default='true')
    p.add_argument('--camera-sensor-profile', choices=('legacy','rgbd_v1'), default='legacy')
    a=p.parse_args()
    command=['xacro', a.model, 'use_sim_time:='+a.use_sim_time]
    if a.camera_sensor_profile!='legacy':
        command.append('camera_sensor_profile:='+a.camera_sensor_profile)
    if a.profile=='r5_support_v1':command.append('suspension_profile:=independent_linear_v1')
    text=subprocess.check_output(command, text=True)
    if a.profile=='r5_support_v1':
        import support_profile
        support_profile.verify(a.root)
        text=support_profile.augment_urdf(text,a.root)
    if a.profile=='r5_v2':
        verify(a.root)
        text=augment_urdf(text,a.root)
    print(text,end='')

if __name__=='__main__':main()
