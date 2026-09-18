"""Pre-action Nav2 path snapshots for controlled observation experiments; no motion commands."""
import base64
import math
import time
import numpy as np


def check_path(points,goal,grid):
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not 2<=len(points)<=1000 or not np.isfinite(points).all():
        raise ValueError('Expected a finite Nav2 XY/yaw path')
    if np.linalg.norm(points[-1,:2]-goal)>.25:raise ValueError('Nav2 path does not reach the requested goal')
    resolution=grid['resolution'];width,height=grid['width'],grid['height']
    costs=np.frombuffer(base64.b64decode(grid['data']),dtype=np.int8).reshape(height,width)
    xy=points[:,:2];lengths=np.linalg.norm(np.diff(xy,axis=0),axis=1)
    if lengths.sum()>50:raise ValueError('Observation path exceeds 50m offline limit')
    for a,b,length in zip(xy,xy[1:],lengths):
        for t in np.linspace(0,1,max(2,math.ceil(length/(resolution/2))+1)):
            ix,iy=np.floor((a+t*(b-a)-grid['origin'])/resolution).astype(int)
            if not 0<=ix<width or not 0<=iy<height or not 0<=costs[iy,ix]<99:
                raise ValueError('Path crosses unknown, occupied or out-of-map space in the pre-action map')
    return points.tolist()


def compute_path(node,goal,state,observation_yaw):
    import rclpy
    from rclpy.action import ActionClient
    from nav2_msgs.action import ComputePathToPose
    from simulation_experiment import within_region
    motion=state['motion'];pose=motion['pose'];task=state['task']
    if motion['active'] or not motion['ready']:raise ValueError('Planning requires a stationary, ready experiment')
    if not within_region(task['region'],*goal,observation_yaw):raise ValueError('Observation pose outside task boundary')
    client=ActionClient(node,ComputePathToPose,'/compute_path_to_pose');handle=None
    def wait(future):
        deadline=time.monotonic()+15
        while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not future.done():raise TimeoutError('Nav2 path query exceeded 15 wall seconds')
        return future.result()
    try:
        if not client.wait_for_server(timeout_sec=5):raise ValueError('Nav2 planner is not ready')
        request=ComputePathToPose.Goal();request.use_start=True;request.planner_id='GridBased'
        for item,x,y,yaw in ((request.start,pose['x'],pose['y'],pose['yaw']),(request.goal,*goal,observation_yaw)):
            item.header.frame_id='map';item.header.stamp=node.get_clock().now().to_msg()
            item.pose.position.x=float(x);item.pose.position.y=float(y)
            item.pose.orientation.z=math.sin(yaw/2);item.pose.orientation.w=math.cos(yaw/2)
        started=node.get_clock().now().nanoseconds/1e9
        handle=wait(client.send_goal_async(request))
        if not handle.accepted:raise ValueError('Nav2 rejected the path query')
        result=wait(handle.get_result_async())
        if result.status!=4 or result.result.error_code:
            raise ValueError(f'Nav2 planning failed: status={result.status}, code={result.result.error_code}, {result.result.error_msg}')
        path=result.result.path
        if path.header.frame_id!='map':raise ValueError('Expected Nav2 map coordinates')
        points=[]
        for item in path.poses:
            p,q=item.pose.position,item.pose.orientation
            points.append([p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))])
        return dict(experiment=task['id'],query_started_sim_stamp=started,query_completed_sim_stamp=node.get_clock().now().nanoseconds/1e9,
            start_pose=pose,goal=goal,observation_yaw=observation_yaw,path=check_path(points,goal,motion['navigation_grid']),
            map_snapshot=motion['navigation_grid'],path_stamp=path.header.stamp.sec+path.header.stamp.nanosec/1e9,
            meaning='Pre-action Nav2 route snapshot; NavigateToPose may replan during execution; yaw fields are planner poses, not guaranteed camera headings')
    except Exception:
        if handle is not None and handle.accepted:handle.cancel_goal_async()
        raise
    finally:client.destroy()
