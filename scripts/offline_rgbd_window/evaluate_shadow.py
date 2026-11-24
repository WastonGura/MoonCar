"""Evaluate CLOSED shadow JSONL, without ROS, an estimator, or fitted alignment.

The denominator is every declared output slot (legacy default 500 ms), anchored
at the first observed exact RGBD pair. Missing records, queue replacements and
failed solves and stale results
remain visible. Only an explicit, fixed IMU-to-world yaw convention and truth at
the declared gauge exposure set the evaluation coordinates. No result is fed
back to the shadow node. Conditional model C is never augmented with baseline C.
"""
import argparse
from bisect import bisect_left
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

sys.dont_write_bytecode = True
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from offline_rgbd_window.data import interpolate_truth, load_truth

NS = 1_000_000_000
PERIOD_NS = 500_000_000
FRESH_SIM_S = .3
FUTURE_SIM_S = .05
FRESH_WALL_S = .5
MOVING_M_S = .02
TURNING_RAD_S = .05
MOTION_GAP_NS = 50_000_000


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def integer(value, label):
    if type(value) is not int or value < 0:
        raise ValueError(label + ' must be an exact nonnegative integer')
    return value


def vector(value, size, label):
    answer = np.asarray(value, dtype=float)
    if answer.shape != (size,) or not np.isfinite(answer).all():
        raise ValueError('invalid ' + label)
    return answer


def covariance(value):
    C = np.asarray(value, dtype=float)
    if (C.shape != (3, 3) or not np.isfinite(C).all()
            or not np.allclose(C, C.T, atol=1e-10, rtol=1e-8)
            or np.linalg.eigvalsh(C).min() < -1e-10):
        raise ValueError('invalid conditional covariance')
    return C


def stats(values):
    values = [float(v) for v in values if v is not None and math.isfinite(v)]
    return dict(count=len(values), p50=float(np.percentile(values, 50)) if values else None,
                p95=float(np.percentile(values, 95)) if values else None,
                maximum=max(values) if values else None)


def read_rows(path):
    with Path(path).open(encoding='utf-8-sig') as stream:
        return [json.loads(line) for line in stream if line.strip()]


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def _rotation(metadata, imu_world_yaw_offset_rad):
    axes = metadata.get('axes')
    if axes == 'world':
        yaw, reason = 0., 'declared world axes'
    elif axes == 'initial_body_yaw':
        yaw0 = metadata.get('initial_imu_yaw_rad')
        offset = imu_world_yaw_offset_rad
        if offset is None:
            offset = metadata.get('config', {}).get('imu_world_yaw_offset_rad')
        if not finite(yaw0) or not finite(offset):
            return None, dict(axes=axes, available=False,
                reason='initial IMU yaw and explicit IMU-to-world yaw offset are required')
        yaw, reason = yaw0 + offset, 'declared initial IMU yaw plus fixed IMU-to-world offset'
    else:
        return None, dict(axes=axes, available=False, reason='unsupported or missing axis declaration')
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]]), dict(
        axes=axes, available=True, yaw_to_world_rad=yaw, reason=reason,
        initial_imu_yaw_rad=metadata.get('initial_imu_yaw_rad'),
        declared_imu_world_yaw_offset_rad=(offset if axes == 'initial_body_yaw' else None),
        alignment='one declared yaw and translation at the gauge; no fitted rotation, scale or trajectory')


def _baseline(rows):
    """Only exact-time map->base records; never interpolate a TF result."""
    accepted, rejected, duplicates = {}, Counter(), 0
    for row in rows:
        stamp = row.get('stamp_ns')
        if type(stamp) is not int or row.get('lookup_tf_stamp_ns') != stamp:
            rejected['not_exact_lookup_stamp'] += 1
            continue
        point = vector(row.get('map_xyz'), 3, 'baseline map_xyz')
        if stamp in accepted:
            if not np.array_equal(accepted[stamp], point):
                raise ValueError('conflicting exact TF at the same exposure')
            duplicates += 1
        accepted[stamp] = point
    return accepted, dict(exact_records=len(accepted), duplicate_records=duplicates,
                          rejected_records=dict(rejected), interpolation=False)


def _yaw(row):
    if finite(row.get('yaw')):
        return row['yaw']
    q = row.get('q')
    if not isinstance(q, list) or len(q) != 4 or not all(finite(v) for v in q):
        return None
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-12:
        return None
    x, y, z, w = (v / norm for v in q)
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _motion(truth, stamps, left, right):
    times = [r['stamp_ns'] for r in truth]
    velocity, rates, valid_time, moving_time, turning_time, path = {}, {}, 0., 0., 0., 0.
    yaw_path, gaps = 0., []
    for before, after in zip(truth, truth[1:]):
        a, b = before['stamp_ns'], after['stamp_ns']
        dt = b - a
        overlap = max(0, min(b, right) - max(a, left)) / NS
        speed = turn = None
        if 0 < dt <= MOTION_GAP_NS:
            distance = math.dist(before['xyz'][:2], after['xyz'][:2])
            speed = distance * NS / dt
            y0, y1 = _yaw(before), _yaw(after)
            angle = None if y0 is None or y1 is None else abs(math.atan2(math.sin(y1-y0), math.cos(y1-y0)))
            turn = None if angle is None else angle * NS / dt
            if overlap:
                valid_time += overlap
                path += distance * overlap / (dt / NS)
                moving_time += overlap if speed > MOVING_M_S else 0.
                turning_time += overlap if turn is not None and turn > TURNING_RAD_S else 0.
                yaw_path += (angle or 0.) * overlap / (dt / NS)
        elif overlap:
            gaps.append(dict(start_sim=a/NS, end_sim=b/NS, overlap_sim_s=overlap))
        velocity[b], rates[b] = speed, turn
    masks = []
    for stamp in stamps:
        at = bisect_left(times, stamp)
        # At an exact row use its backward interval; otherwise its enclosing interval.
        key = times[at] if at < len(times) else None
        speed, rate = velocity.get(key), rates.get(key)
        masks.append(dict(speed_m_s=speed, absolute_yaw_rate_rad_s=rate,
            moving=None if speed is None else speed > MOVING_M_S,
            turning=None if rate is None else rate > TURNING_RAD_S,
            pure_turn=None if speed is None or rate is None else speed <= MOVING_M_S and rate > TURNING_RAD_S))
    return masks, dict(raw_truth_samples=sum(left <= t <= right for t in times),
        path_xy_m=path, absolute_yaw_change_rad=yaw_path,
        classified_truth_duration_sim_s=valid_time, moving_duration_sim_s=moving_time,
        turning_duration_sim_s=turning_time, truth_gap_intervals=gaps,
        thresholds=dict(moving_gt_m_s=MOVING_M_S, turning_gt_rad_s=TURNING_RAD_S,
                        maximum_truth_velocity_gap_s=MOTION_GAP_NS/NS))


def _gap(times, left, right):
    inside = sorted(set(t for t in times if left <= t <= right))
    return dict(events=len(inside), intervals_sim_s=stats((b-a)/NS for a, b in zip(inside, inside[1:])),
                longest_no_new_output_sim_s=max((b-a)/NS for a, b in zip([left]+inside, inside+[right])))


def _online_fresh(outputs, left, right):
    """Integrate freshness after real delivery; old stamps cannot refresh it."""
    events = sorted((r['ready_sim_ns'], r['stamp_ns']) for r in outputs
                    if r.get('ready_sim_ns') is not None)
    intervals, last_stamp, repeated = [], None, 0
    for ready, stamp in events:
        if last_stamp is not None and stamp <= last_stamp:
            repeated += 1
            continue
        last_stamp = stamp
        if stamp-ready > round(FUTURE_SIM_S*NS):
            continue
        start, end = max(left, ready), min(right, stamp+round(FRESH_SIM_S*NS))
        if start < end:
            intervals.append((start, end))
    merged = []
    for start, end in sorted(intervals):
        if merged and start <= merged[-1][1]:
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
        else:
            merged.append((start, end))
    duration = max(0, right-left)
    fresh = sum(b-a for a, b in merged)
    edges, previous = [], left
    for a, b in merged:
        edges.append(max(0, a-previous))
        previous = max(previous, b)
    edges.append(max(0, right-previous))
    return dict(fresh_sim_seconds=fresh/NS, observed_window_sim_seconds=duration/NS,
        fresh_sim_fraction=fresh/duration if duration else None,
        longest_not_fresh_sim_seconds=max(edges, default=0)/NS,
        repeated_or_nonincreasing_exposure_deliveries=repeated,
        scope='retrospective integration of result-delivery clock and original exposure stamps; not a navigation-health verdict')


def evaluate(shadow, truth, baseline=(), *, epoch=None, start_ns=None, end_ns=None,
             imu_world_yaw_offset_rad=None):
    """Pure closed-record statistics. Returns (summary, all fixed-slot records)."""
    epochs = {row.get('epoch') for row in shadow if row.get('epoch') is not None}
    if epoch is None:
        if len(epochs) != 1:
            raise ValueError('one explicit shadow epoch is required')
        epoch = next(iter(epochs))
    rows = [row for row in shadow if row.get('epoch') == epoch]
    if not rows:
        raise ValueError('no records for requested epoch')
    metadata = {}
    for row in rows:
        if row.get('type') in ('run_start', 'imu_reference', 'gauge', 'run_end'):
            metadata.update({key: value for key, value in row.items() if value is not None})
    interval = metadata.get('config', {}).get('interval_ns', PERIOD_NS)
    integer(interval, 'config interval_ns')
    if interval == 0:
        raise ValueError('config interval_ns must be positive')
    keyframe_interval = metadata.get('config', {}).get('keyframe_interval_ns')
    if keyframe_interval is not None:
        integer(keyframe_interval, 'config keyframe_interval_ns')
        if keyframe_interval == 0:
            raise ValueError('config keyframe_interval_ns must be positive')
    exposure, dispositions = {}, Counter()
    plans = {}
    for row in rows:
        if row.get('type') == 'exposure':
            stamp = integer(row.get('stamp_ns'), 'exposure stamp_ns')
            exposure[stamp] = row
            dispositions[row.get('disposition', 'unspecified')] += 1
        elif row.get('type') == 'plan':
            stamp = integer(row.get('planned_stamp_ns'), 'planned_stamp_ns')
            if stamp in plans:
                raise ValueError('duplicate plan terminal record')
            plans[stamp] = row
    if not exposure:
        raise ValueError('no independently recorded same-stamp RGBD pair')
    anchor = min(exposure)
    for stamp in plans:
        if stamp < anchor or (stamp-anchor) % interval:
            raise ValueError('plan is not on the declared first-pair output grid')
    left = anchor if start_ns is None else integer(start_ns, 'start_ns')
    observed_end = max([anchor, *exposure, *plans,
                        *(r['sim_ns'] for r in rows if type(r.get('sim_ns')) is int)])
    right = observed_end if end_ns is None else integer(end_ns, 'end_ns')
    if right < left:
        raise ValueError('end precedes start')
    first = anchor + max(0, (left-anchor+interval-1)//interval)*interval
    expected = list(range(first, right+1, interval))
    normalized, output_stamps, gauges = [], set(), set()
    for due in expected:
        row = dict(plans.get(due, dict(planned_stamp_ns=due, stamp_ns=None, output=False,
                                      reason='missing_plan_record')))
        if type(row.get('output')) is not bool:
            raise ValueError('every plan needs explicit boolean output')
        stamp = row.get('stamp_ns')
        if stamp is not None:
            integer(stamp, 'plan exposure stamp_ns')
            if row['output'] and not due <= stamp < due+interval:
                raise ValueError('output exposure is outside its own fixed planning interval')
        if row['output']:
            if stamp not in exposure or stamp in output_stamps:
                raise ValueError('output must reference a unique, actually observed exposure')
            output_stamps.add(stamp)
            row['position'] = vector(row.get('position'), 3, 'shadow position').tolist()
            C = covariance(row.get('covariance_conditional'))
            row['covariance_conditional'] = C.tolist()
            gauge = integer(row.get('gauge_stamp_ns', metadata.get('gauge_stamp_ns')), 'gauge_stamp_ns')
            gauges.add(gauge)
            ready = row.get('ready_sim_ns')
            if ready is not None:
                integer(ready, 'ready_sim_ns')
            age = None if ready is None else (ready-stamp)/NS
            if row.get('age_sim_s') is not None and (not finite(row['age_sim_s'])
                    or age is None or abs(row['age_sim_s']-age) > 1e-8):
                raise ValueError('age disagrees with original exposure and result-ready clock')
            row['age_sim_s'] = age
        times = [row.get(name) for name in ('received_wall_ns', 'started_wall_ns', 'finished_wall_ns')]
        for value in times:
            if value is not None:
                integer(value, 'monotonic wall ns')
        if all(value is not None for value in times):
            received, started, finished = times
            if not received <= started <= finished:
                raise ValueError('noncausal queue/compute wall times')
            row.update(queue_wait_wall_s=(started-received)/NS,
                       compute_wall_s=(finished-started)/NS, end_to_end_wall_s=(finished-received)/NS)
        normalized.append(row)
    if metadata.get('gauge_stamp_ns') is not None:
        gauges.add(integer(metadata['gauge_stamp_ns'], 'metadata gauge'))
    if len(gauges) > 1:
        raise ValueError('gauge changed inside one shadow epoch; reanchoring cannot be hidden')
    gauge = next(iter(gauges)) if gauges else None
    if gauge is not None and gauge not in exposure:
        raise ValueError('gauge is not an actually observed RGBD exposure')
    rotation, coordinate = _rotation(metadata, imu_world_yaw_offset_rad)
    query = [r['stamp_ns'] if r.get('stamp_ns') is not None else r['planned_stamp_ns'] for r in normalized]
    values, valid = interpolate_truth(truth, query)
    origin, origin_valid = interpolate_truth(truth, [] if gauge is None else [gauge])
    origin_ok = gauge is not None and bool(origin_valid[0])
    coordinate.update(gauge_stamp_ns=gauge, truth_at_gauge_available=origin_ok,
        covariance='conditional C only; no baseline C0 is added; frame-wise noise correlations remain model assumptions')
    baseline_map, baseline_audit = _baseline(baseline)
    masks, motion = _motion(truth, query, left, right)
    for row, actual, usable, mask, stamp in zip(normalized, values, valid, masks, query):
        row.update(truth_available=bool(usable), motion=mask, baseline_exact_tf=stamp in baseline_map)
        row['error_xy_m'] = row['baseline_error_xy_m'] = None
        if usable and stamp in baseline_map:
            row['baseline_error_xy_m'] = float(np.linalg.norm(baseline_map[stamp][:2]-actual[:2]))
        if row['output']:
            C = np.asarray(row['covariance_conditional'])
            world_C = C if rotation is None else rotation @ C @ rotation.T
            row['conditional_xy_sigma_m'] = math.sqrt(max(0., float(np.linalg.eigvalsh(world_C[:2, :2]).max())))
            row['fresh_sim_at_delivery'] = row['age_sim_s'] is not None and -FUTURE_SIM_S <= row['age_sim_s'] <= FRESH_SIM_S+1e-12
            row['fresh_sim_and_wall_at_delivery'] = row['fresh_sim_at_delivery'] and row.get('end_to_end_wall_s') is not None and row['end_to_end_wall_s'] <= FRESH_WALL_S
            row['delivered_within_evaluation_window'] = row.get('ready_sim_ns') is not None and left <= row['ready_sim_ns'] <= right
            if usable and origin_ok and rotation is not None:
                error = rotation @ np.asarray(row['position']) - (actual-origin[0])
                row['error_xy_m'] = float(np.linalg.norm(error[:2]))
    outputs = [r for r in normalized if r['output']]
    common = [r for r in outputs if r['error_xy_m'] is not None and r['baseline_error_xy_m'] is not None]
    selected_raw = {t: row for t, row in exposure.items() if left <= t <= right}
    final_dispositions = Counter(row.get('disposition', 'unspecified') for row in selected_raw.values())
    failure_reasons = Counter(r.get('reason', 'unspecified') for r in normalized if not r['output'])
    def fraction(count):
        return count/len(normalized) if normalized else None
    def subset(predicate):
        sample = [r for r in normalized if predicate(r)]
        return dict(plans=len(sample), outputs=sum(r['output'] for r in sample),
            conditional_error_xy_m=stats(r['error_xy_m'] for r in sample),
            baseline_exact_tf=sum(r['baseline_exact_tf'] for r in sample))
    maximum_failures = run = 0
    for row in normalized:
        run = 0 if row['output'] else run+1
        maximum_failures = max(maximum_failures, run)
    all_truth = [r for r in truth if left <= r['stamp_ns'] <= right]
    summary = dict(schema='rgbd-window-shadow-evaluation-v1', epoch=epoch,
        capture_closed=any(r.get('type') == 'run_end' and r.get('complete') is True
                           and r.get('worker_terminal') is True for r in rows),
        recorded_final_counts=metadata.get('counts'),
        recorded_fault_events=[{k: r.get(k) for k in ('reason', 'sim_ns', 'wall_ns')}
                               for r in rows if r.get('type') == 'fault'],
        window_sim=[left/NS, right/NS], denominator=dict(anchor_stamp_ns=anchor,
            interval_sim_s=interval/NS,
            declared_keyframe_interval_sim_s=None if keyframe_interval is None else keyframe_interval/NS,
            expected_plans=len(normalized),
            missing_plan_records=failure_reasons.get('missing_plan_record', 0),
            source='every declared output slot at/after the first observed exact RGBD pair; keyframes and queries count once each; no output/age/truth filtering'),
        raw_rgbd=dict(unique_observed_pairs=len(selected_raw), terminal_dispositions=dict(final_dispositions),
            dropped_replaced_or_rejected=sum(final_dispositions.get(k, 0) for k in ('replaced', 'rejected')),
            unfinished_queued=final_dispositions.get('queued', 0), all_disposition_events=dict(dispositions)),
        shadow=dict(outputs=len(outputs), no_output=len(normalized)-len(outputs),
            outputs_delivered_within_window=sum(r['delivered_within_evaluation_window'] for r in outputs),
            outputs_delivered_after_window=sum(r.get('ready_sim_ns') is not None and r['ready_sim_ns'] > right for r in outputs),
            outputs_with_unknown_delivery_time=sum(r.get('ready_sim_ns') is None for r in outputs),
            output_fraction_of_all_plans=fraction(len(outputs)), failure_reasons=dict(failure_reasons),
            maximum_consecutive_failed_plans=maximum_failures,
            conditional_sigma_xy_m=stats(r.get('conditional_xy_sigma_m') for r in outputs),
            conditional_sigma_within_50mm=sum(r['conditional_xy_sigma_m'] <= .05 for r in outputs),
            conditional_sigma_within_50mm_fraction_of_all_plans=fraction(sum(r['conditional_xy_sigma_m'] <= .05 for r in outputs)),
            truth_scored_outputs=sum(r['error_xy_m'] is not None for r in outputs),
            conditional_error_xy_m=stats(r['error_xy_m'] for r in outputs),
            stale_output_error_xy_m=stats(r['error_xy_m'] for r in outputs
                if r['age_sim_s'] is not None and not r['fresh_sim_at_delivery']),
            unknown_delivery_age_error_xy_m=stats(r['error_xy_m'] for r in outputs if r['age_sim_s'] is None),
            fresh_sim_at_delivery=sum(r['fresh_sim_at_delivery'] for r in outputs),
            fresh_deliveries_within_window=sum(r['fresh_sim_at_delivery'] and r['delivered_within_evaluation_window'] for r in outputs),
            fresh_sim_fraction_of_all_plans=fraction(sum(r['fresh_sim_at_delivery'] for r in outputs)),
            fresh_sim_and_wall_at_delivery=sum(r['fresh_sim_and_wall_at_delivery'] for r in outputs)),
        common_exact_baseline=dict(samples=len(common), stamp_range_ns=[common[0]['stamp_ns'], common[-1]['stamp_ns']] if common else None,
            shadow_error_xy_m=stats(r['error_xy_m'] for r in common),
            baseline_error_xy_m=stats(r['baseline_error_xy_m'] for r in common),
            scope='only real same-stamp exact TF and truth; conditional comparison, not the main availability denominator'),
        baseline=dict(**baseline_audit, exact_tf_at_plan_evaluation_stamp=sum(r['baseline_exact_tf'] for r in normalized),
            missing_tf_at_plan_evaluation_stamp=sum(not r['baseline_exact_tf'] for r in normalized),
            all_raw_truth_denominator=len(all_truth), exact_tf_raw_truth_samples=sum(r['stamp_ns'] in baseline_map for r in all_truth)),
        timing=dict(queue_wait_wall_s=stats(r.get('queue_wait_wall_s') for r in normalized),
            compute_wall_s=stats(r.get('compute_wall_s') for r in normalized),
            end_to_end_wall_s=stats(r.get('end_to_end_wall_s') for r in normalized),
            exposure_age_at_result_ready_sim_s=stats(r.get('age_sim_s') for r in outputs),
            frontend_wall_s=stats(r.get('frontend_s') for r in normalized),
            backend_wall_s=stats(r.get('backend_s') for r in normalized),
            scope='per planned exposure monotonic wall latency; not CPU time or all raw-frame total compute'),
        gaps=dict(exposure_stamp=_gap([r['stamp_ns'] for r in outputs], left, right),
            result_delivery_sim=_gap([r['ready_sim_ns'] for r in outputs if r.get('ready_sim_ns') is not None], left, right)),
        online_freshness=_online_fresh(outputs, left, right), coordinate_contract=coordinate,
        physical=motion, moving_plans=subset(lambda r: r['motion']['moving'] is True),
        turning_plans=subset(lambda r: r['motion']['turning'] is True),
        pure_turn_plans=subset(lambda r: r['motion']['pure_turn'] is True),
        thresholds=dict(fresh_sim_max_s=FRESH_SIM_S, tolerated_future_phase_s=FUTURE_SIM_S,
            result_end_to_end_wall_reference_s=FRESH_WALL_S, model_xy_sigma_reference_m=.05),
        limitations=['No ROS/navigation health or capability PASS is inferred.',
            'Late outputs are retained in error statistics and cannot refresh their exposure timestamps.',
            'Conditional model covariance is not a calibrated global posterior.',
            'Exact TF availability can be retrospective; it does not prove baseline online availability.',
            'Initialization translation sets a gauge; its zero error is not an accuracy improvement.'])
    return summary, normalized


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--shadow', required=True, type=Path)
    parser.add_argument('--truth', required=True, type=Path)
    parser.add_argument('--baseline', type=Path, help='Closed observer/alignment.jsonl or the identical exact-TF schema')
    parser.add_argument('--epoch', help='Required when a file contains multiple shadow epochs')
    parser.add_argument('--start-sim', type=float)
    parser.add_argument('--end-sim', type=float)
    parser.add_argument('--imu-world-yaw-offset-rad', type=float,
                        help='Declared sensor/setup yaw offset, never fitted from truth')
    parser.add_argument('--output', required=True, type=Path, help='A new directory; existing evidence is not overwritten')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output directory already exists')
    for name in ('start_sim', 'end_sim', 'imu_world_yaw_offset_rad'):
        value = getattr(args, name)
        if value is not None and not math.isfinite(value):
            parser.error(name + ' must be finite')
    sources = [Path(__file__).resolve(), Path(__file__).with_name('data.py').resolve()]
    inputs = [args.shadow, args.truth]+([args.baseline] if args.baseline else [])
    before = {str(p.resolve()): digest(p) for p in inputs+sources}
    summary, plans = evaluate(read_rows(args.shadow), load_truth(args.truth),
        read_rows(args.baseline) if args.baseline else (), epoch=args.epoch,
        start_ns=None if args.start_sim is None else round(args.start_sim*NS),
        end_ns=None if args.end_sim is None else round(args.end_sim*NS),
        imu_world_yaw_offset_rad=args.imu_world_yaw_offset_rad)
    after = {str(p.resolve()): digest(p) for p in inputs+sources}
    if before != after:
        raise ValueError('input or evaluator source changed during closed-record evaluation')
    args.output.mkdir(parents=True)
    (args.output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (args.output/'plans.jsonl').write_text(''.join(json.dumps(row, allow_nan=False)+'\n' for row in plans), encoding='utf-8')
    integrity = dict(input_and_source_sha256=before, unchanged=True,
        output_sha256={name: digest(args.output/name) for name in ('summary.json', 'plans.jsonl')})
    (args.output/'integrity.json').write_text(json.dumps(integrity, indent=2)+'\n', encoding='utf-8')
    print(json.dumps(dict(output=str(args.output.resolve()), plans=len(plans), outputs=summary['shadow']['outputs'])))


if __name__ == '__main__':
    main()
