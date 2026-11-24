"""Known answers, denominator integrity, reference failure and acceptance boundaries."""
import copy
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
from mapping_acceptance import build_reference, score
from report_mapping_acceptance import prepare,rebuild,matrices
from survey_evaluation import write_json
from terrain_geometry import project_depth


class AcceptanceTest(unittest.TestCase):
    def test_known_maps_and_evidence(self):
        reference=dict(resolution=.25,reference_audit=dict(passed=True),cells=[
            dict(ix=i,iy=0,x=(i+.5)*.25,y=.125,z=0.,slope=0.,category='ground') for i in range(100)])
        estimates=[dict(ix=i,iy=0,z=0. if i<72 else .1,slope=0.,valid=True) for i in range(80)]
        evidence=dict(coordinate_contract_verified=True,sensor_lineage_verified=True,cutoff_verified=True)
        rows,s=score(reference,estimates,evidence=evidence)
        self.assertEqual((s['C'],s['P'],s['S']),(.8,.9,72))
        self.assertAlmostEqual(s['height_rmse_m'],math.sqrt(8*.1**2/80))
        self.assertEqual(s['status'],'geometry_pass_safety_unverified')
        self.assertEqual(sum(r['state']=='error_exceeds_threshold' for r in rows),8)
        for cells,c,p,total in (([],0,None,0),(estimates[:20],.2,1,20),
                                ([dict(c,z=.1) for c in estimates],.8,0,0),
                                ([dict(c,slope=10.) for c in estimates],.8,0,0),
                                ([dict(c,ix=c['ix']+100) for c in estimates],0,None,0)):
            _,s=score(reference,cells,evidence=evidence)
            self.assertEqual((s['C'],s['P'],s['S']),(c,p,total))
        invalid=estimates+[dict(ix=80,iy=0,z=None,slope=None,valid=False)]
        _,s=score(reference,invalid,evidence=evidence)
        self.assertEqual((s['C_obs'],s['C']),(.81,.8))
        perfect=[dict(ix=i,iy=0,z=0.,slope=0.,valid=True) for i in range(100)]
        _,s=score(reference,perfect,evidence=dict(evidence,safety_pass=True))
        self.assertEqual(s['status'],'passed_under_simulation_profile')
        _,s=score(reference,perfect,evidence=dict(evidence,runtime_fault=True,safety_pass=True))
        self.assertEqual((s['S'],s['status']),(100,'failed'))
        for field in evidence:
            rows,s=score(reference,perfect,evidence=dict(evidence,**{field:False}))
            self.assertEqual(s['status'],'not_evaluable');self.assertIsNone(s['S'])
            self.assertTrue(all(r['height_error_m'] is None and r['qualified'] is None for r in rows))
        bad=copy.deepcopy(reference);bad['reference_audit']['passed']=False
        self.assertIsNone(score(bad,perfect,evidence=evidence)[1]['P'])
        # Large ground region cannot hide failed non-flat terrain.
        strata=copy.deepcopy(reference);strata['cells'][-1]['category']='terrain'
        _,s=score(strata,perfect[:-1],evidence=evidence)
        self.assertEqual(s['S'],99);self.assertFalse(s['geometry_pass'])
        excluded=copy.deepcopy(reference);excluded['cells'][-1]['category']='edge_or_complex'
        _,s=score(excluded,perfect,evidence=evidence)
        self.assertEqual((s['S'],s['representation_fraction'],s['excluded_area_m2']),(100,.99,.0625))
        for bad_cells in (perfect+[perfect[0]], [dict(perfect[0],z=float('nan'))],
                          [dict(perfect[0],valid=1)], [dict(perfect[0],slope=91)]):
            with self.assertRaises(ValueError):score(reference,bad_cells,evidence=evidence)
        wrong_grid=copy.deepcopy(reference);wrong_grid['cells'][0]['x']+=.1
        with self.assertRaises(ValueError):score(wrong_grid,perfect,evidence=evidence)
        # Independently known horizontal geometry, also exercises dense-reference convergence.
        ref=build_reference([('ground','plane',np.array([50.,50.]),np.eye(4))],[0,0],.5)
        self.assertTrue(ref['reference_audit']['passed'])
        self.assertTrue(all(abs(c['z'])<1e-12 and abs(c['slope'])<1e-12 for c in ref['cells']))
        ramp=copy.deepcopy(reference)
        for c in ramp['cells']:c.update(z=.1*c['ix'],slope=math.degrees(math.atan(.4)),category='terrain')
        shifted=[dict(ix=c['ix']+4,iy=0,z=c['z'],slope=c['slope'],valid=True) for c in ramp['cells']]
        _,s=score(ramp,shifted,evidence=evidence)
        self.assertEqual((s['C'],s['S']),(.96,0));self.assertAlmostEqual(s['height_rmse_m'],.4)

    def test_runner_domain_is_checked_before_start(self):
        from capture_slope_resampling import run
        with patch.dict('os.environ',{'ROS_DOMAIN_ID':'0'}):
            with self.assertRaisesRegex(ValueError,'ROS_DOMAIN_ID=89'):run(Path('must-not-create'))

    def test_real_input_adapter_guards(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);capture=root/'capture';capture.mkdir()
            world=root/'world.sdf'
            world.write_text('<sdf><world><model name="ground"><static>true</static><link><visual><geometry><plane><size>50 50</size></plane></geometry></visual></link></model></world></sdf>')
            ref=prepare(world,root/'reference',[0,0],.5,5)
            (capture/'survey_yard.sdf').write_bytes(world.read_bytes())
            raw=np.ones((64,64),dtype='<f4');fmt=dict(width=64,height=64,step=256,encoding='32FC1',bigendian=False)
            k=[50.,0,32.,0,50.,32.,0,0,1.]
            cloud=project_depth(raw.tobytes(),64,64,256,'32FC1',False,[50,50,32,32],pixel_offset=.5).astype(np.float32)
            # The frame after the frozen deadline must not enter the reconstructed map.
            stamps=np.array([1.,5.9,6.1]);poses=np.repeat(np.eye(4)[None],3,axis=0)
            np.savez_compressed(capture/'samples.npz',stamp=stamps,points=np.tile(cloud,(3,1)),
                                offsets=np.arange(4)*len(cloud),estimated_camera=poses)
            np.savez_compressed(capture/'raw-depth.npz',stamps=stamps,data=np.tile(np.frombuffer(raw.tobytes(),dtype=np.uint8),(3,1)))
            write_json(capture/'depth-format.json',fmt);write_json(capture/'camera-info.json',dict(k=k))
            write_json(capture/'manifest.json',dict(status='captured',frames=3,first_stamp=1.,last_stamp=6.1,
                                                   seconds=5,projection_pixel_offset=.5))
            model,baseline,audit=rebuild(capture,ref)
            self.assertEqual(len(model.frames),2);self.assertEqual(audit['ignored_after_cutoff'],1)
            self.assertTrue(audit['sensor_lineage_verified']);self.assertFalse(audit['coordinate_contract_verified'])
            self.assertEqual(score(ref,model.elevation(),evidence=audit)[1]['status'],'not_evaluable')
            # Same-sized raw images with wrong values cannot validate a sensor lineage.
            np.savez_compressed(capture/'raw-depth.npz',stamps=stamps,data=np.zeros((3,raw.nbytes),dtype=np.uint8))
            with self.assertRaisesRegex(ValueError,'does not reproduce'):rebuild(capture,ref)
            with self.assertRaises(ValueError):prepare(world,root/'reference',[0,0],.5,5)
            broken=poses.copy();broken[0,0,0]=2
            with self.assertRaises(ValueError):matrices(broken,3,'bad rotation')


if __name__=='__main__':unittest.main()
