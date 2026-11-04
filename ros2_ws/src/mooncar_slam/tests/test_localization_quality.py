"""Quality regression fixtures independent of ROS, Gazebo and matching weights."""
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / 'scripts'))
from localization_quality import LaserQualitySource, LocalizationQuality


def fitted(source, stamp):
    source.observe(stamp, dict(accepted=True, reason='accepted'))


def ready_source(session='first'):
    source = LaserQualitySource(session)
    source.observe(0., dict(accepted=True, reason='initialized'))
    for stamp in (.1, .2, .3):
        fitted(source, stamp)
    return source


def ready_consumer():
    source = ready_source()
    consumer = LocalizationQuality(required=True)
    consumer.observe(source.snapshot(.3), 10.)
    return source, consumer


class LaserQualitySourceTests(unittest.TestCase):
    def test_initialization_requires_three_genuine_fits(self):
        source = LaserQualitySource('fixture')
        source.observe(0., dict(accepted=True, reason='initialized'))
        self.assertEqual(source.snapshot(0.)['successful_streak'], 0)
        for stamp in (.1, .2):
            fitted(source, stamp)
            self.assertFalse(source.snapshot(stamp)['healthy'])
        fitted(source, .3)
        payload = source.snapshot(.3)
        self.assertTrue(payload['healthy'])
        self.assertEqual(payload['ready_since_stamp'], .3)

    def test_stationary_throttle_keeps_recent_valid_measurement(self):
        source = ready_source()
        source.observe(.4, dict(accepted=False, reason='stationary_hold'))
        payload = source.snapshot(.4)
        self.assertTrue(payload['healthy'])
        self.assertEqual(payload['accepted_stamp'], .3)
        self.assertEqual(payload['ready_since_stamp'], .3)

    def test_each_rejection_immediately_invalidates_then_requires_three_fits(self):
        for reason in ('too_few_line_correspondences', 'unsynchronized', 'too_few_returns',
                       'nonplanar_mount', 'pending_overflow', 'nonfinite_input'):
            with self.subTest(reason=reason):
                source = ready_source()
                source.observe(.4, dict(accepted=False, reason=reason))
                payload = source.snapshot(.4)
                self.assertFalse(payload['healthy'])
                self.assertEqual(payload['reason'], reason)
                self.assertIsNone(payload['ready_since_stamp'])
                for stamp in (.5, .6):
                    fitted(source, stamp)
                    self.assertFalse(source.snapshot(stamp)['healthy'])
                fitted(source, .7)
                payload = source.snapshot(.7)
                self.assertTrue(payload['healthy'])
                self.assertEqual(payload['ready_since_stamp'], .7)

    def test_missing_non_scan_input_invalidates_without_changing_scan_epoch(self):
        source = ready_source()
        source.reject('imu_transform_unavailable')
        self.assertFalse(source.snapshot(.3)['healthy'])
        self.assertEqual(source.last_input_stamp, .3)

    def test_fresh_heartbeat_cannot_replace_missing_fits(self):
        source = ready_source()
        self.assertTrue(source.snapshot(.6)['healthy'])
        payload = source.snapshot(.6001)
        self.assertFalse(payload['healthy'])
        self.assertEqual(payload['reason'], 'observation_stale')
        self.assertEqual(payload['successful_streak'], 0)
        fitted(source, .7)
        self.assertFalse(source.snapshot(.7)['healthy'])

    def test_fit_after_a_large_unobserved_gap_is_only_first_recovery_fit(self):
        source = ready_source()
        fitted(source, .7)
        payload = source.snapshot(.7)
        self.assertFalse(payload['healthy'])
        self.assertEqual(payload['successful_streak'], 1)

    def test_duplicate_scan_is_rejected_and_cannot_add_recovery_success(self):
        source = ready_source()
        fitted(source, .3)
        payload = source.snapshot(.3)
        self.assertFalse(payload['healthy'])
        self.assertEqual(payload['reason'], 'nonincreasing_scan_stamp')

    def test_backwards_input_and_backwards_clock_latch_source_fault(self):
        for mode in ('input', 'clock'):
            with self.subTest(mode=mode):
                source = ready_source()
                source.snapshot(.3)
                if mode == 'input':
                    fitted(source, .1)
                else:
                    source.snapshot(.1)
                for stamp in (.4, .5, .6):
                    fitted(source, stamp)
                payload = source.snapshot(.6)
                self.assertFalse(payload['healthy'])
                self.assertEqual(payload['reason'], 'clock_moved_backwards')

    def test_bad_inputs_and_results_never_count_as_success(self):
        for stamp, result in ((float('nan'), dict(accepted=True, reason='accepted')),
                              (.4, {}), (.4, dict(accepted=1, reason='accepted'))):
            with self.subTest(stamp=stamp, result=result):
                source = ready_source()
                source.observe(stamp, result)
                self.assertFalse(source.snapshot(.4)['healthy'])

    def test_snapshot_sequences_increase_even_while_simulation_is_paused(self):
        source = ready_source()
        first, second = source.snapshot(.3), source.snapshot(.3)
        self.assertEqual(second['sequence'], first['sequence'] + 1)
        self.assertEqual(first['stamp'], second['stamp'])


class LocalizationQualityTests(unittest.TestCase):
    def test_disabled_baseline_requires_no_producer(self):
        quality = LocalizationQuality()
        quality.observe({}, float('nan'))
        self.assertEqual(quality.check(0., 0.)['reason'], 'disabled')
        self.assertTrue(quality.check(0., 0.)['healthy'])

    def test_required_quality_fails_closed_before_first_status(self):
        self.assertEqual(LocalizationQuality(True).check(0., 0.)['reason'], 'status_missing')

    def test_status_wall_and_measurement_sim_freshness_have_separate_boundaries(self):
        source, quality = ready_consumer()
        self.assertTrue(quality.check(.6, 10.5)['healthy'])
        self.assertEqual(quality.check(.6001, 10.5)['reason'], 'observation_stale')
        source, quality = ready_consumer()
        self.assertTrue(quality.check(.3, 10.5)['healthy'])
        self.assertEqual(quality.check(.3, 10.5001)['reason'], 'status_stale')

    def test_heartbeat_with_old_observation_does_not_make_position_healthy(self):
        source, quality = ready_consumer()
        quality.observe(source.snapshot(.6001), 10.2)
        self.assertEqual(quality.check(.6001, 10.2)['reason'], 'observation_stale')

    def test_status_timeout_requires_three_new_measurements_and_moves_frame_epoch(self):
        source, quality = ready_consumer()
        self.assertEqual(quality.check(.3, 10.501)['reason'], 'status_stale')
        quality.observe(source.snapshot(.3), 10.6)
        self.assertEqual(quality.check(.3, 10.6)['reason'], 'status_recovering')
        for stamp in (.4, .5):
            fitted(source, stamp)
            quality.observe(source.snapshot(stamp), 10.7)
            self.assertFalse(quality.check(stamp, 10.7)['healthy'])
        fitted(source, .6)
        quality.observe(source.snapshot(.6), 10.8)
        self.assertTrue(quality.check(.6, 10.8)['healthy'])
        self.assertEqual(quality.check(.6, 10.8, observation_stamp=.3)['reason'], 'observation_before_ready')

    def test_measurement_timeout_cannot_be_healed_by_a_held_heartbeat(self):
        source, quality = ready_consumer()
        self.assertEqual(quality.check(.601, 10.1)['reason'], 'observation_stale')
        quality.observe(source.snapshot(.3), 10.2)
        self.assertEqual(quality.check(.601, 10.2)['reason'], 'status_recovering')
        for stamp in (.61, .71):
            fitted(source, stamp)
            quality.observe(source.snapshot(stamp), 10.3)
            self.assertFalse(quality.check(stamp, 10.3)['healthy'])
        fitted(source, .81)
        quality.observe(source.snapshot(.81), 10.4)
        self.assertTrue(quality.check(.81, 10.4)['healthy'])
        self.assertEqual(quality.check(.81, 10.4, observation_stamp=.3)['reason'], 'observation_before_ready')

    def test_duplicate_or_reordered_status_is_invalid_and_does_not_refresh_receipt(self):
        source, quality = ready_consumer()
        payload = dict(quality.payload)
        quality.observe(payload, 10.4)
        self.assertEqual(quality.received_wall, 10.)
        self.assertEqual(quality.check(.3, 10.4)['reason'], 'nonincreasing_status_sequence')
        quality.observe(source.snapshot(.3), 10.45)
        self.assertEqual(quality.check(.3, 10.45)['reason'], 'status_recovering')

    def test_invalid_payload_requires_three_advancing_measurements_to_recover(self):
        source, quality = ready_consumer()
        quality.observe({}, 10.1)
        self.assertEqual(quality.check(.3, 10.1)['reason'], 'invalid_status')
        self.assertEqual(quality.received_wall, 10.)
        quality.observe(source.snapshot(.3), 10.2)
        self.assertEqual(quality.check(.3, 10.2)['reason'], 'status_recovering')
        for stamp in (.4, .5):
            fitted(source, stamp)
            quality.observe(source.snapshot(stamp), 10.2)
            self.assertFalse(quality.check(stamp, 10.2)['healthy'])
        fitted(source, .6)
        quality.observe(source.snapshot(.6), 10.2)
        self.assertTrue(quality.check(.6, 10.2)['healthy'])
        self.assertEqual(quality.check(.6, 10.2, observation_stamp=.3)['reason'], 'observation_before_ready')

    def test_schema_rejects_nonfinite_and_wrongly_typed_fields(self):
        source = ready_source()
        valid = source.snapshot(.3)
        for key, value in (('schema_version', True), ('session_id', ''), ('sequence', True),
                           ('sequence', 0), ('stamp', float('nan')), ('stamp', 10 ** 1000), ('healthy', 1),
                           ('accepted_stamp', float('inf')), ('ready_since_stamp', float('nan')),
                           ('ready_since_stamp', .5), ('successful_streak', False),
                           ('successful_streak', 0), ('reason', '')):
            with self.subTest(key=key, value=value):
                quality = LocalizationQuality(True)
                quality.observe(dict(valid, **{key: value}), 10.)
                self.assertEqual(quality.check(.3, 10.)['reason'], 'invalid_status')

    def test_producer_restart_remains_blocked_until_explicit_consumer_reset(self):
        source, quality = ready_consumer()
        restarted = ready_source('replacement')
        quality.observe(restarted.snapshot(.3), 10.1)
        self.assertEqual(quality.check(.3, 10.1)['reason'], 'producer_session_changed')
        quality.observe(source.snapshot(.3), 10.2)
        self.assertFalse(quality.check(.3, 10.2)['healthy'])
        quality.reset()
        quality.observe(restarted.snapshot(.3), 10.3)
        self.assertTrue(quality.check(.3, 10.3)['healthy'])

    def test_consumer_clock_rollback_latches_even_if_status_later_becomes_fresh(self):
        source, quality = ready_consumer()
        self.assertTrue(quality.check(.3, 10.)['healthy'])
        self.assertEqual(quality.check(.2, 10.1)['reason'], 'clock_moved_backwards')
        quality.observe(source.snapshot(.4), 10.2)
        self.assertFalse(quality.check(.4, 10.2)['healthy'])
        quality.reset()
        quality.observe(source.snapshot(.4), 10.3)
        self.assertTrue(quality.check(.4, 10.3)['healthy'])

    def test_source_clock_rollback_is_detected_despite_increasing_sequence(self):
        source, quality = ready_consumer()
        payload = dict(quality.payload, sequence=2, stamp=.25)
        quality.observe(payload, 10.1)
        self.assertEqual(quality.check(.3, 10.1)['reason'], 'clock_moved_backwards')

    def test_bounded_future_clock_skew_is_allowed_but_larger_skew_blocks(self):
        source = ready_source()
        payload = source.snapshot(.3)
        quality = LocalizationQuality(True)
        quality.observe(payload, 10.)
        self.assertTrue(quality.check(.2, 10.)['healthy'])
        quality = LocalizationQuality(True)
        quality.observe(payload, 10.)
        self.assertEqual(quality.check(.199, 10.)['reason'], 'status_from_future')

    def test_frame_window_uses_ready_epoch_and_accepted_measurement_limit(self):
        source, quality = ready_consumer()
        self.assertEqual(quality.check(.3, 10., observation_stamp=.299)['reason'], 'observation_before_ready')
        self.assertTrue(quality.check(.3, 10., observation_stamp=.3)['healthy'])
        self.assertTrue(quality.check(.3, 10., observation_stamp=.4)['healthy'])
        self.assertEqual(quality.check(.3, 10., observation_stamp=.401)['reason'], 'observation_after_measurement')
        self.assertEqual(quality.check(.3, 10., observation_stamp=float('nan'))['reason'], 'invalid_observation_stamp')

    def test_pre_fault_frames_cannot_reopen_after_recovery(self):
        source, quality = ready_consumer()
        source.observe(.4, dict(accepted=False, reason='too_few_line_correspondences'))
        quality.observe(source.snapshot(.4), 10.1)
        self.assertFalse(quality.check(.4, 10.1, observation_stamp=.3)['healthy'])
        for stamp in (.5, .6, .7):
            fitted(source, stamp)
        quality.observe(source.snapshot(.7), 10.2)
        self.assertEqual(quality.check(.7, 10.2, observation_stamp=.3)['reason'], 'observation_before_ready')
        self.assertTrue(quality.check(.7, 10.2, observation_stamp=.7)['healthy'])

    def test_missed_unhealthy_status_still_exposes_changed_recovery_epoch(self):
        source, quality = ready_consumer()
        source.observe(.4, dict(accepted=False, reason='too_few_line_correspondences'))
        # A callback drains several scans before the consumer receives status;
        # only the healthy end of that batch reaches the consumer.
        for stamp in (.5, .6, .7):
            fitted(source, stamp)
        quality.observe(source.snapshot(.7), 10.2)
        self.assertEqual(quality.check(.7, 10.2)['reason'], 'recovery_epoch_changed')
        self.assertFalse(quality.check(.7, 10.2)['healthy'])
        quality.observe(source.snapshot(.7), 10.3)
        self.assertTrue(quality.check(.7, 10.3)['healthy'])
        self.assertEqual(quality.check(.7, 10.3, observation_stamp=.3)['reason'], 'observation_before_ready')

    def test_recovery_epoch_cannot_move_backwards(self):
        source, quality = ready_consumer()
        payload = dict(quality.payload, sequence=2, ready_since_stamp=.2)
        quality.observe(payload, 10.1)
        self.assertEqual(quality.check(.3, 10.1)['reason'], 'recovery_epoch_moved_backwards')

    def test_paused_ros_clock_is_valid_while_steady_heartbeat_continues(self):
        source, quality = ready_consumer()
        quality.observe(source.snapshot(.3), 11.)
        self.assertTrue(quality.check(.3, 11.)['healthy'])
        self.assertFalse(quality.check(.3, 11.501)['healthy'])


if __name__ == '__main__':
    unittest.main()
