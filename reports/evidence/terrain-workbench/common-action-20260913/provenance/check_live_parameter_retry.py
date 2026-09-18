"""Isolated ROS90 service delays its first reply; verify the real timeout/retry path."""
import sys,time,threading,json
from pathlib import Path
sys.path.insert(0,'/source/scripts')
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue
from trajectory_evaluation import read_pixel_offset
from survey_evaluation import write_json
rclpy.init(domain_id=90);server=Node('capture_retry_fixture');node=Node('capture_retry_client')
calls=[]
def respond(request,response):
    calls.append(time.monotonic())
    if len(calls)==1:time.sleep(11)
    response.values=[ParameterValue(type=3,double_value=.5)];return response
server.create_service(GetParameters,'/capture_retry_fixture/delayed_values',respond,callback_group=ReentrantCallbackGroup())
executor=MultiThreadedExecutor(num_threads=2);executor.add_node(server)
worker=threading.Thread(target=executor.spin);worker.start()
client=node.create_client(GetParameters,'/capture_retry_fixture/delayed_values')
class Adapter:
    def wait_for_services(self,timeout_sec):return client.wait_for_service(timeout_sec=timeout_sec)
    def get_parameters(self,names):return client.call_async(GetParameters.Request(names=names))
events=[]
try:
    value=read_pixel_offset(node,Adapter(),events)
    assert value==.5 and [e['status'] for e in events]==['response_timeout','success'] and len(calls)==2
    # Drain the deliberately late first response before destroying the fixture.
    while time.monotonic()-calls[0]<12:rclpy.spin_once(node,timeout_sec=.05)
    write_json(Path(sys.argv[1]),dict(domain=90,events=events,requests=len(calls),value=value,passed=True))
    print(json.dumps(dict(events=events,requests=len(calls),passed=True)))
finally:
    executor.shutdown();worker.join();server.destroy_node();node.destroy_node();rclpy.shutdown()
