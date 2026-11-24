"""Accept complete live library bindings and retain incomplete startup samples."""
import copy,json,time
from pathlib import Path
class Readiness:
    def __init__(self,path,limit=8192):
        self.path=Path(path);self.stream=self.path.open('x',buffering=1)
        self.limit=limit;self.count=0;self.pid=None;self.accepted=None
    def observe(self,value):
        if value is None:return None
        if self.count>=self.limit:raise RuntimeError('Bounded library receipt history overflow')
        if self.pid is not None and value.get('pid')!=self.pid:raise RuntimeError('Gazebo PID changed during readiness')
        self.pid=value['pid'];self.count+=1
        self.stream.write(json.dumps(dict(sample=self.count,receive_wall=time.time(),receipt=value))+'\n')
        self.stream.flush()
        if value.get('passed') is True:self.accepted=copy.deepcopy(value)
        return self.accepted
    def close(self):self.stream.close()
