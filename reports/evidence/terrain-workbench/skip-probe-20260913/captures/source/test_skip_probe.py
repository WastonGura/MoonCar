"""A prefix is immutable, sensor-only, and selection never examines later labels."""
import json
from pathlib import Path
import sys
import tempfile
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
import numpy as np
from trajectory_evaluation import save_sensor_prefix
from skip_probe import select,DELTAS

if __name__=='__main__':
    with tempfile.TemporaryDirectory() as directory:
        folder=Path(directory)
        frames=[dict(stamp=1.,estimated_camera=np.eye(4),estimated_robot=np.eye(4),truth_robot='never export')]
        save_sensor_prefix(folder,frames,[np.ones((2,3))],{'width':320},.5)
        with np.load(folder/'checkpoint.npz',allow_pickle=False) as data:
            assert set(data.files)=={'points','offsets','stamp','estimated_camera','estimated_robot'}
            assert data['stamp'].tolist()==[1.]
        assert json.loads((folder/'checkpoint.json').read_text())['frames']==1
    replies=[([{'marginal_prediction':True}],{'reason':'try_observation'}),
             ([{'marginal_prediction':False}],{'reason':'no_modelled_marginal_support'})]
    with patch('skip_probe.predict',side_effect=replies):
        chosen,rows=select({},None,np.eye(4),np.eye(4),{},.5,10)
        assert chosen['delta']==DELTAS[1] and len(rows)==2
    for response in [([],{'reason':'no_modelled_marginal_support'}),
                     ([{'marginal_prediction':False}],{'reason':'insufficient_budget'})]:
        with patch('skip_probe.predict',return_value=response):
            chosen,rows=select({},None,np.eye(4),np.eye(4),{},.5,10)
            assert chosen is None and len(rows)==len(DELTAS)
    print('PASS sensor-only prefix, frozen first-negative selection, no empty/budget false admission')
