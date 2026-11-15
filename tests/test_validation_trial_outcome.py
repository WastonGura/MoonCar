import errno
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
from unittest.mock import Mock
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts/validation'))
from trial_outcome import lifecycle_snapshot, mission_outcome


class TrialOutcomeTests(unittest.TestCase):
    def test_normal_budget_stop_is_distinct_from_failure(self):
        self.assertEqual(mission_outcome('budget_reached'), 'budget_completed')
        self.assertEqual(mission_outcome('completed'), 'passed')
        for phase in ('fault', 'localization_paused', 'manual', None):
            self.assertEqual(mission_outcome(phase), 'failed')

    def test_transient_snapshot_error_is_recorded_and_cannot_open_readiness(self):
        errors = []
        with patch.object(Path, 'read_text', side_effect=OSError(errno.ENODATA, 'No data available')):
            value = lifecycle_snapshot('/witness/latest.json', errors)
        self.assertFalse(value['all_active'])
        self.assertEqual(len(errors), 1)
        self.assertIn('No data available', errors[0]['error'])
        with patch.object(Path, 'read_text', return_value='{"all_active": true}'):
            self.assertTrue(lifecycle_snapshot('/witness/latest.json', errors)['all_active'])

    def test_lost_lifecycle_reply_can_be_retried_without_claiming_ready(self):
        from observe_lifecycle import collect_responses
        client=Mock()
        future=Mock()
        future.done.return_value=False
        pending={'controller_server':(future,1.)}
        states={}
        collect_responses({'controller_server':client},pending,states,2.)
        self.assertIn('controller_server',pending)
        collect_responses({'controller_server':client},pending,states,3.)
        self.assertEqual(pending,{})
        self.assertNotIn('id',states['controller_server'])
        client.remove_pending_request.assert_called_once_with(future)
        future.cancel.assert_called_once()
        reply=Mock()
        reply.done.return_value=True
        reply.result.return_value=SimpleNamespace(current_state=SimpleNamespace(id=3,label='active'))
        pending['controller_server']=(reply,3.)
        collect_responses({'controller_server':client},pending,states,3.2)
        self.assertEqual(states['controller_server']['id'],3)


if __name__ == '__main__': unittest.main()
