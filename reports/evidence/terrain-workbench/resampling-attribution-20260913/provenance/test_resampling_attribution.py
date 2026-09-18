from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from resampling_attribution import phase_indices,subset_indices


def check():
    poses=np.repeat(np.eye(4)[None],7,axis=0);poses[:,0,3]=[0,0,0,.2,.4,.5,.5]
    stamps=np.arange(7.)
    assert phase_indices(stamps,poses,2.,2)==(3,5)
    assert phase_indices(stamps,poses,2.,2,stationary=True)==(2,2)
    parts=subset_indices(7,2,3,5)
    assert parts['waiting']==[0,1,2] and parts['transit']==[0,1,3,4]
    assert parts['settled']==[0,1,5,6] and parts['final_frame']==[0,1,6]
    for indices in parts.values():assert indices==sorted(set(indices))
    poses[:]=np.eye(4)
    assert phase_indices(stamps,poses,2.,2)==(7,7)
    print('PASS: phase partition, shared baseline, no duplicate frames and no-motion detection')


if __name__=='__main__':check()
