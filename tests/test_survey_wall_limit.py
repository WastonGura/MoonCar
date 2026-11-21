"""Long simulation collection must not silently retain the one-hour run cap."""
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from autonomous_survey import AutonomousSurvey


def mission(limit=None):
    motion = Mock()
    motion.lock = threading.RLock()
    motion.state.return_value = dict(active=False, ready=True, pose={'x': 0., 'y': 0.}, diagnostics={})
    motion.quality = {}
    motion.quality_received = 4000.
    motion.grid_fresh.return_value = True
    survey = AutonomousSurvey(
        motion,
        localization_revisit_enabled=False,
        **({} if limit is None else {'wall_limit_seconds': limit}),
    )
    survey.data.update(active=True, started=0., budget_seconds=480., radius=None)
    survey.wall_started = 0.
    survey.origin = [0., 0.]
    survey.now = Mock(return_value=400.)
    survey.stop = Mock()
    survey.update_localization_revisit = Mock()
    return survey


class SurveyWallLimitTests(unittest.TestCase):
    def test_default_cap_still_stops_after_one_wall_hour(self):
        survey = mission()
        with patch('autonomous_survey.time.monotonic', return_value=4000.):
            self.assertFalse(survey._healthy_once())
        survey.stop.assert_called_once()

    def test_explicit_long_run_cap_allows_eight_minute_simulation_to_continue(self):
        survey = mission(7200.)
        with patch('autonomous_survey.time.monotonic', return_value=4000.):
            self.assertTrue(survey._healthy_once())
        survey.stop.assert_not_called()
        survey.now.return_value = 480.
        self.assertFalse(survey._healthy_once())
        self.assertEqual(survey.stop.call_args.args[1], 'budget_reached')

    def test_extended_cap_is_still_finite_and_enforced(self):
        survey = mission(7200.)
        with patch('autonomous_survey.time.monotonic', return_value=7201.):
            self.assertFalse(survey._healthy_once())
        survey.stop.assert_called_once()
        for value in (True, '7200', float('nan'), float('inf'), 0., 86401.):
            with self.assertRaises(ValueError):
                AutonomousSurvey(Mock(), wall_limit_seconds=value)


if __name__ == '__main__':
    unittest.main()
