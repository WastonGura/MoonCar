"""Offline checks: fixed denominators, matched gain scope, evidence/report completeness."""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
import subprocess
sys.path.insert(0,str(Path(__file__).resolve().parents[4]/'scripts'))
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from survey_evaluation import metrics,gain,reference_errors,report,summarize,write_json,VERSION
from terrain_map import TerrainMap
import numpy as np


class EvaluationTests(unittest.TestCase):
    def test_scope_and_gain(self):
        before=dict(resolution=1.,cells=[dict(ix=0,iy=0,quality=.5)],frames=1,stamp=1)
        after=dict(resolution=1.,cells=[dict(ix=0,iy=0,quality=.8),dict(ix=1,iy=0,quality=.4),dict(ix=9,iy=9,quality=1.)],frames=2,stamp=2)
        a,b=metrics(before,[0,0],2),metrics(after,[0,0],2)
        self.assertEqual(a['roi_area_m2'],12)
        self.assertEqual(a['roi_area_m2'],b['roi_area_m2'])
        self.assertEqual(b['observed_area_m2'],2)
        delta=gain(before,after,[0,0],2,{(0,0)})
        self.assertAlmostEqual(delta['quality_gain_m2'],.3)
        self.assertEqual(delta['new_observed_m2'],0)
        self.assertIsNone(delta['recovered_surface_m2'])
        before['cells'][0]['fit_state']='too_few_voxels'
        for cell in after['cells']:cell['fit_state']='valid'
        recovered=gain(before,after,[0,0],2)
        self.assertEqual(recovered['resampling_eligible_cells'],1)
        self.assertEqual(recovered['resampling_recovered_cells'],1)
        self.assertEqual(recovered['recovered_surface_m2'],1.)
        self.assertAlmostEqual(gain(before,after,[0,0],2)['quality_gain_m2'],.7)
        self.assertEqual(gain(after,after,[0,0],2)['quality_gain_m2'],0)
        self.assertIsNone(metrics(dict(resolution=1.,cells=[]),[0,0],2)['mean_observed_quality'])

    def test_reference_requires_alignment_and_reports_no_overlap(self):
        cells=[dict(ix=0,iy=0,z=1.,slope=10.,valid=True)]
        ref=dict(frame='map',resolution=.25,source='analytic test',alignment_description='identity; synthetic',cells=[dict(ix=0,iy=0,z=.9,slope=8.)])
        result=reference_errors(cells,ref,.25,'map')
        self.assertAlmostEqual(result['height_rmse_m'],.1);self.assertEqual(result['slope_mae_deg'],2)
        with self.assertRaises(ValueError):reference_errors(cells,ref,.5,'map')
        self.assertIsNone(reference_errors([],ref,.25,'map')['height_rmse_m'])

    def test_report_preserves_failed_actions_raw_and_figures(self):
        with tempfile.TemporaryDirectory() as root:
            folder=Path(root)/'experiment-test/research/autonomy-test';folder.mkdir(parents=True)
            terrain=TerrainMap()
            points=np.array([[x,y,0.] for x in np.arange(.01,.25,.06) for y in np.arange(.01,.25,.06)])
            terrain.add_frame(points,1.,robot_translation=(0,0,0),robot_quaternion=(0,0,0,1))
            start=terrain.quality()
            terrain.add_frame(points-[1,0,0],2.,translation=(1,0,0),robot_translation=(1,0,0),robot_quaternion=(0,0,0,1))
            end=terrain.quality();terrain.save(folder/'observations.npz')
            mission=dict(phase='fault',origin=[0,0],radius=2,budget_seconds=15,elapsed_seconds=2,reason='test sensor failure',history=[dict(success=False,outcome='interrupted')])
            write_json(folder/'manifest.json',dict(version=VERSION,run_id='autonomy-test',mission=mission,strategy='synthetic',status='captured',limitations=['synthetic test only']))
            write_json(folder/'mission.json',mission)
            rows=[]
            for i,q in enumerate((start,end)):
                label=('start','end')[i];write_json(folder/f'quality-{label}.json',q)
                rows.append(dict(elapsed_seconds=i+1,sim_stamp=i+1,path_m=float(i),pose=dict(x=i,y=0),**metrics(q,[0,0],2)))
            (folder/'telemetry.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in rows),encoding='utf-8')
            result=report(folder)
            self.assertEqual(result['failed_actions'],1);self.assertEqual(result['status'],'fault')
            self.assertIsNone(result['height_rmse_m'])
            self.assertIsNone(result['resampling_eligible_cells'])
            self.assertIsNone(result['resampling_success_fraction'])
            for name in ('sampling-comparison.png','progress.png','collected-map.png','elevation.csv','actions.csv','hashes.json'):
                self.assertGreater((folder/name).stat().st_size,20)
            self.assertEqual(json.loads((folder/'manifest.json').read_text())['report_status'],'ready')
            summarize(root,Path(root)/'summary')
            self.assertEqual(json.loads((Path(root)/'summary/runs.json').read_text())[0]['status'],'fault')
            # Exercise the real postprocessing entry point: reference stays separate and unaccepted.
            (folder/'survey_yard.sdf').write_text('<sdf><world><model name="ground"><static>true</static><link name="ground"><visual name="surface"><geometry><plane><normal>0 0 1</normal><size>50 50</size></plane></geometry></visual></link></model></world></sdf>')
            subprocess.run([sys.executable,str(Path(__file__).resolve().parents[4]/'scripts/survey_evaluation.py'),
                            'report',str(folder)],check=True,capture_output=True,text=True,timeout=90)
            analyses=list((folder.parent/'reference-analysis').glob('*/summary.json'))
            self.assertEqual(len(analyses),1)
            diagnostic=json.loads(analyses[0].read_text())
            self.assertFalse(diagnostic['accepted_accuracy'])
            self.assertEqual(diagnostic['groups'][0]['matched_cells'],1)
            self.assertAlmostEqual(diagnostic['groups'][0]['height_rmse_m'],0)
            self.assertIsNone(json.loads((folder/'summary.json').read_text())['height_rmse_m'])


if __name__=='__main__':unittest.main()
