"""Verify frozen raw-depth correspondence, residual denominators, mission and hashes."""
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

root=Path(__file__).resolve().parent
for path in root.rglob('hashes.json'):
    for name,digest in json.loads(path.read_text()).items():
        file=path.parent/name
        assert file.resolve().is_relative_to(path.parent.resolve())
        assert hashlib.sha256(file.read_bytes()).hexdigest()==digest,str(file)
sys.path.insert(0,str(root/'fixed-motion'))
from terrain_geometry import project_depth
for name in ('raw-02','fixed-raw-02','fixed-motion'):
    folder=root/name;manifest=json.loads((folder/'manifest.json').read_text())
    assert manifest['status']=='captured'
    assert manifest['dropped_selected_frames']==len(json.loads((folder/'drops.json').read_text()))
    info=json.loads((folder/'camera-info.json').read_text());fmt=json.loads((folder/'depth-format.json').read_text())
    intrinsics=[info['k'][i] for i in (0,4,2,5)]
    with np.load(folder/'samples.npz') as data,np.load(folder/'raw-depth.npz') as raw:
        images=raw['data'];points=data['points'];offsets=data['offsets']
        assert len(data['stamp'])==manifest['frames'] and np.all(data['gap']<=.05)
        np.testing.assert_array_equal(data['stamp'],raw['stamps'])
        np.testing.assert_allclose(data['truth_camera'],data['truth_robot']@manifest['camera_extrinsic'],atol=1e-12)
        for i in range(len(data['stamp'])):
            expected=project_depth(images[i].tobytes(),fmt['width'],fmt['height'],fmt['step'],fmt['encoding'],fmt['bigendian'],intrinsics,pixel_offset=manifest.get('projection_pixel_offset',0)).astype(np.float32)
            np.testing.assert_array_equal(expected,points[offsets[i]:offsets[i+1]])
for analysis,capture in (('raw-analysis',root/'raw-02'),('fixed-raw-analysis',root/'fixed-raw-02'),('fixed-motion-analysis',root/'fixed-motion'),('trajectory-analysis',root.parent/'alignment-20260913/trajectory-01')):
    folder=root/analysis;summary=json.loads((folder/'summary.json').read_text())
    for name,digest in summary['input_sha256'].items():
        assert hashlib.sha256((capture/name).read_bytes()).hexdigest()==digest,(analysis,name)
    with np.load(folder/'residuals.npz') as data:
        for prefix,offset in (('nominal',0.),('half_pixel',.5)):
            errors,owner=data[prefix],data[prefix+'_owner']
            assert len(errors)==summary['observed_points'] and data['offsets'][-1]==len(errors)
            missing_key='nominal_missing_reference' if offset==0 else 'half_pixel_missing_reference'
            assert int((owner<0).sum())==summary[missing_key]
            for category in ('all','plane','solid'):
                mask=np.array([k>=0 and (category=='all' or (summary['shapes'][k]['kind']=='plane')==(category=='plane')) for k in owner])
                values=errors[mask];row=next(r for r in summary['groups'] if r['pixel_offset']==offset and r['category']==category)
                assert len(values)==row['points']
                assert math.isclose(float(np.sqrt(np.mean(values**2))),row['rmse_m'],abs_tol=1e-12)
        assert summary['groups'][3]['rmse_m']<summary['groups'][0]['rmse_m']
    expected_frames=json.loads((capture/'manifest.json').read_text())['frames']
    assert summary['frames']==expected_frames
    assert summary['raw_frames_bitwise_reprojected']==(expected_frames if (capture/'raw-depth.npz').exists() else 0)
mission=json.loads((root/'autonomy-a5c5e2ce5e/mission.json').read_text())
assert not mission['active'] and mission['phase'] in ('no_candidates','completed')
assert any(h['kind']=='advance' and h['success'] for h in mission['history'])
stop=json.loads((root/'stop-check.json').read_text())
assert stop['passed'] and len(stop['safe_commands'])>=5
assert all(abs(x)+abs(z)<1e-9 for x,z in stop['safe_commands'])
print('PASS: raw pixel correspondence, source hashes, residual groups/counts, autonomous translation and stopped velocity')
