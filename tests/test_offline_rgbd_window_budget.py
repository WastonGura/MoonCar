"""Work limits must precede LK and preserve real measurements and guards."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from offline_rgbd_window.frontend import TrackFrontend


def table():
    u,v = np.meshgrid(np.linspace(16,304,30),np.linspace(16,224,30))
    p = np.column_stack((u.ravel(),v.ravel()))
    return dict(pixels=p,points=np.column_stack((p*.01,np.ones(900)*2)),
        covariance=np.tile(np.eye(3)*1e-5,(900,1,1)),pixel_scales=np.ones(900),
        track_ids=np.arange(900),descriptors=np.zeros((900,32),dtype=np.uint8),
        descriptor_valid=np.ones(900,dtype=bool),measured_roi=np.array([0.,0.,320.,240.]),
        image_shape=(240,320),_measured_gray=np.zeros((240,320),dtype=np.uint8))


class BudgetContracts(unittest.TestCase):
    def frontend(self):
        f = TrackFrontend(track_budget=576)
        original = table()
        f.previous = (original,original['track_ids'].copy())
        f.age = {i:10 for i in range(900)}
        return f,original

    def test_real_rows_and_all_protected_ids_survive_when_budget_fits(self):
        f,original = self.frontend()
        f.keyframes.append((dict(submitted_ids=np.arange(200)),np.arange(200)))
        f.protect_submission(1,range(804,900))
        previous = f.previous
        bounded,ids = f._budget_previous()
        self.assertEqual(len(ids),480)
        self.assertTrue(set(range(200))|set(range(804,900)) <= set(ids))
        self.assertEqual(f.last_tracking['budget_kept_protected'],296)
        self.assertEqual(f.last_tracking['budget_replenishment_capacity'],96)
        for key in ('points','pixels','covariance','pixel_scales','descriptors','descriptor_valid'):
            np.testing.assert_array_equal(bounded[key],original[key][ids])
        self.assertIs(f.previous,previous)
        self.assertEqual(len(original['track_ids']),900)
        self.assertEqual(f.measurements._orb.getMaxFeatures(),1400)
        cells = (bounded['pixels']/[320,240]*[8,6]).astype(int)
        self.assertEqual(len(set(map(tuple,cells))),48)

    def test_overflow_is_hard_bounded_and_not_claimed_as_full_protection(self):
        f,_ = self.frontend()
        f.keyframes.append((dict(submitted_ids=np.arange(800)),np.arange(800)))
        f.protect_submission(1,range(804,900))
        _,ids = f._budget_previous()
        self.assertEqual(len(ids),576)
        self.assertEqual(len(set(ids)),576)
        self.assertEqual(f.last_tracking['budget_protected'],896)
        self.assertEqual(f.last_tracking['budget_kept_protected'],576)
        self.assertEqual(f.last_tracking['budget_replenishment_capacity'],0)

    def test_running_and_latest_pending_are_bounded_and_completion_clears(self):
        f,_ = self.frontend()
        for request in (1,2,3): f.protect_submission(request,[request])
        self.assertEqual(set(f._tracking_submissions),{1,2,3})
        f.protect_submission(4,[4])
        self.assertEqual(set(f._tracking_submissions),{1,3,4})
        f.complete_submissions(1)
        self.assertEqual(set(f._tracking_submissions),{3,4})
        f.complete_submissions(4)
        self.assertFalse(f._tracking_submissions)
        for bad in (0,119,1401,True,576.):
            with self.assertRaises(ValueError): TrackFrontend(track_budget=bad)

    def test_failed_job_handoff_between_poll_and_offer_keeps_successor_ids(self):
        f,_ = self.frontend()
        f.protect_submission(1,[101])
        f.protect_submission(2,[202])
        # The last poll saw job1 running. It now fails, job2 starts, and the
        # same-version offer3 is accepted before the foreground polls again.
        f.protect_submission(3,[303])
        self.assertIn(202,set().union(*f._tracking_submissions.values()))
        f.complete_submissions(1)
        self.assertEqual(set(f._tracking_submissions),{2,3})
        f.protect_submission(4,[404])
        self.assertIn(202,set().union(*f._tracking_submissions.values()))
        self.assertLessEqual(len(f._tracking_submissions),3)

    def test_lk_input_is_bounded_before_native_work_and_checks_still_pass(self):
        rng = np.random.default_rng(89)
        image = cv2.GaussianBlur(rng.integers(0,256,(240,320),dtype=np.uint8),(3,3),.6)
        v,u = np.indices(image.shape)
        grid = np.stack(((u+.5-160)*.01,(v+.5-120)*.01,np.full(image.shape,2.)),axis=2)
        f = TrackFrontend(track_budget=144)
        first = f.process(SimpleNamespace(gray=image,xyz_grid=grid))
        selected,diag = f.select(first)
        self.assertTrue(diag['adequate'])
        f.commit(first,selected)
        old = first['pixels'].copy()
        moved = cv2.warpAffine(image,np.array([[1.,0.,.2],[0.,1.,0.]]),(320,240),borderMode=cv2.BORDER_REFLECT)
        calls = []
        real = cv2.calcOpticalFlowPyrLK
        def checked(*args,**kwargs):
            calls.append(len(args[2]))
            return real(*args,**kwargs)
        with patch.object(cv2,'calcOpticalFlowPyrLK',side_effect=checked):
            current = f.process(SimpleNamespace(gray=moved,xyz_grid=grid))
        self.assertEqual(len(calls),2)
        self.assertTrue(all(n<=96 for n in calls))
        self.assertLessEqual(len(current['track_ids']),144)
        _,diag = f.select(current)
        self.assertTrue(diag['adequate'])
        self.assertGreaterEqual(diag['connected'],24)
        np.testing.assert_array_equal(first['pixels'],old)
        previous = f.previous
        with patch.object(f,'_track',side_effect=RuntimeError('injected')):
            with self.assertRaises(RuntimeError): f.process(SimpleNamespace(gray=image,xyz_grid=grid))
        self.assertIs(f.previous,previous)


if __name__=='__main__': unittest.main()
