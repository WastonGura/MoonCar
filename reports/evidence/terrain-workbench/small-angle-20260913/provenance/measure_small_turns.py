import sys,json,math
from pathlib import Path
import numpy as np
sys.path.insert(0,'/source/scripts')
from survey_evaluation import write_json
captures=Path(sys.argv[1]);output=Path(sys.argv[2]);plan=json.loads((captures/'plan.json').read_text());rows=[]
for trial in plan['trials']:
    if trial['arm']!='turn':continue
    folder=captures/trial['folder'];c=json.loads((folder/'approach-control.json').read_text())
    if 'observe_turn_terminal' not in c:continue
    with np.load(folder/'approach/samples.npz',allow_pickle=False) as data:
        stamp=data['stamp'];start=c['observation_decision_sim_stamp'];end=c['observe_turn_terminal']['sim_stamp']
        i=int(np.searchsorted(stamp,start,side='right'))-1;j=int(np.searchsorted(stamp,end+.5))
        if not 0<=i<j<len(stamp):continue
        row=dict(folder=trial['folder'],requested=trial['observation_delta'],backend_estimated_delta=c.get('observation_actual_delta'),before_stamp=float(stamp[i]),after_stamp=float(stamp[j]),decision_stamp=start,turn_end_stamp=end)
        for key in ('estimated_robot','truth_robot'):
            pose=data[key];a=math.atan2(pose[i,1,0],pose[i,0,0]);b=math.atan2(pose[j,1,0],pose[j,0,0]);row[key+'_delta']=math.atan2(math.sin(b-a),math.cos(b-a))
        rows.append(row)
write_json(output/'rotation-audit.json',dict(rows=rows,meaning='Evaluator-only Gazebo truth and estimated yaw change between bracketing sensor frames; includes pre/post margin, not exact action interval; never a prediction input or retrospective acceptance change.'))
print(json.dumps(rows,indent=2))
