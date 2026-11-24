"""Replay existing sensor archives through an unchanged baseline and new map.

Truth is opened only AFTER estimation has finished. The fixed 2 Hz planned
output denominator includes all failed attempts. No ROS/workbench imports.
"""
import argparse
from collections import Counter
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

sys.dont_write_bytecode = True
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import cv2
import numpy as np

from offline_rgbd_window.core import Frame, Observation, WindowEstimator, WindowError
from offline_rgbd_window.data import iter_sensor_frames, load_truth, interpolate_truth
from offline_rgbd_window.frontend import TrackFrontend, SLAM_SCRIPTS
from metric_rgbd_odometry import MetricRGBDOdometry


def plain(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    return value


def write_json(path, value):
    Path(path).write_text(json.dumps(plain(value), ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


class BaselineCache:
    """A sealed complete baseline, consumed once at exact sensor timestamps.

    Its positions never enter the new estimator.  Only the unchanged evaluation
    origin and one-time C0 sensitivity comparison use the initial cached row.
    """
    def __init__(self, directory, manifest_path):
        self.directory = Path(directory).resolve()
        self.run_path = self.directory/'run.json'
        self.rows_path = self.directory/'baseline.jsonl'
        self.run_sha = digest(self.run_path)
        prior = json.loads(self.run_path.read_text(encoding='utf-8-sig'))
        before, after = prior.get('source_before'), prior.get('source_after')
        if not before or before != after:
            raise ValueError('Cached baseline source seal is missing or changed')
        core_entries = [value for path, value in before.items()
                        if str(path).replace('\\', '/').endswith('/metric_rgbd_odometry.py')]
        current_core_sha = digest(SLAM_SCRIPTS/'metric_rgbd_odometry.py')
        if len(core_entries) != 1 or core_entries[0] != current_core_sha:
            raise ValueError('Cached baseline metric_rgbd_odometry source SHA differs')
        manifest_path = Path(manifest_path)
        manifest_sha = digest(manifest_path)
        if prior.get('manifest_sha256') != manifest_sha:
            raise ValueError('Cached baseline manifest SHA differs')
        manifest = json.loads(manifest_path.read_text(encoding='utf-8-sig'))
        count = manifest.get('extracted_frames')
        if type(count) is not int or count <= 0 or prior.get('sensor_frames') != count:
            raise ValueError('Cached baseline does not cover the complete manifest count')
        if prior.get('config', {}).get('max_sensor_frames', 0) != 0:
            raise ValueError('A truncated cached baseline cannot be reused')
        self.rows_sha = digest(self.rows_path)
        if prior.get('output_sha256', {}).get('baseline.jsonl') != self.rows_sha:
            raise ValueError('Cached baseline JSONL SHA differs')
        with self.rows_path.open(encoding='utf-8-sig') as stream:
            self.rows = [json.loads(line) for line in stream if line.strip()]
        if len(self.rows) != count:
            raise ValueError('Cached baseline row count differs from the full manifest')
        last = None
        for row in self.rows:
            stamp = row.get('stamp_ns')
            if type(stamp) is not int or stamp < 0 or (last is not None and stamp <= last):
                raise ValueError('Cached baseline timestamps must be unique and strictly increasing')
            if type(row.get('output')) is not bool or not isinstance(row.get('reason'), str):
                raise ValueError('Cached baseline output status is invalid')
            elapsed = row.get('baseline_s')
            if type(elapsed) not in (int, float) or not math.isfinite(elapsed) or elapsed < 0:
                raise ValueError('Cached baseline historical timing is invalid')
            if row['output']:
                position = np.asarray(row.get('position'), dtype=float)
                covariance = np.asarray(row.get('covariance'), dtype=float)
                if (position.shape != (3,) or not np.isfinite(position).all()
                        or covariance.shape != (3, 3) or not np.isfinite(covariance).all()
                        or not np.allclose(covariance, covariance.T, atol=1e-10, rtol=1e-8)
                        or float(np.linalg.eigvalsh(covariance).min()) <= 0):
                    raise ValueError('Cached baseline position or covariance is invalid')
            last = stamp
        self.cursor = 0
        self.metadata = dict(directory=str(self.directory), run_sha256=self.run_sha,
            baseline_jsonl_sha256=self.rows_sha, baseline_source_sha256=current_core_sha,
            manifest_sha256=manifest_sha, sensor_frames=count,
            historical_baseline_seconds_p50_p95_max=percentiles([row['baseline_s'] for row in self.rows]),
            historical_baseline_total_compute_seconds=sum(row['baseline_s'] for row in self.rows),
            baseline_s_scope='Historical perf_counter wall measurements from the sealed prior run; '
                             'not recomputed or timed as baseline work in this run.',
            estimator_input_scope='Cached poses are evaluation-only; initial C0 is added once after solving, never passed to WindowEstimator.')

    def row_for(self, stamp_ns):
        if self.cursor >= len(self.rows) or self.rows[self.cursor]['stamp_ns'] != stamp_ns:
            raise ValueError('Cached baseline does not exactly match the next sensor timestamp')
        row = copy.deepcopy(self.rows[self.cursor])
        self.cursor += 1
        row['baseline_cached'] = True
        row['baseline_s_scope'] = 'historical_prior_run_wall_measurement'
        return row

    def finish(self):
        if self.cursor != len(self.rows):
            raise ValueError('Cached baseline has unused tail rows')
        if digest(self.run_path) != self.run_sha or digest(self.rows_path) != self.rows_sha:
            raise ValueError('Cached baseline changed during replay')


def sigma_xy(C):
    return float(np.sqrt(np.linalg.eigvalsh(np.asarray(C)[:2, :2]).max()))


def percentiles(values):
    return None if not len(values) else np.percentile(values, [50, 95, 100]).tolist()


def score(rows, truth_values, truth_valid, origin, covariance_key='covariance'):
    errors, nees, coverage = [], [], []
    # Model availability and uncertainty do not depend on an external truth
    # recorder covering the timestamp. Only error/coverage scoring does.
    sigmas = [sigma_xy(row[covariance_key]) for row in rows if row.get('output', False)]
    stamps, path_xyz, covered_stamps = [], [], []
    for row, xyz, valid in zip(rows, truth_values, truth_valid):
        if not valid:
            continue
        path_xyz.append(xyz)
        covered_stamps.append(row['stamp_ns'])
        if not row.get('output', False):
            continue
        e = np.asarray(row['position']) - (xyz-origin)
        C = np.asarray(row[covariance_key])
        errors.append(float(np.linalg.norm(e[:2])))
        n = float(e[:2] @ np.linalg.solve(C[:2, :2], e[:2]))
        nees.append(n)
        coverage.append(n <= 5.991464547107979)
        stamps.append(row['stamp_ns'])
    return dict(planned=len(rows), outputs=sum(bool(r.get('output')) for r in rows),
                truth_covered_plans=len(path_xyz), scored_outputs=len(errors),
                error_xy_p50_p95_max_m=percentiles(errors),
                sigma_xy_p50_p95_max_m=percentiles(sigmas),
                sigma_xy_first_last_m=[sigmas[0], sigmas[-1]] if sigmas else None,
                output_within_50mm=sum(s <= .05 for s in sigmas),
                output_within_50mm_fraction_of_all_plans=sum(s <= .05 for s in sigmas)/len(rows) if rows else 0.,
                xy_95pct_ellipse_coverage=float(np.mean(coverage)) if coverage else None,
                nees_xy_p50_p95_max=percentiles(nees),
                scored_start_end_ns=[stamps[0], stamps[-1]] if stamps else None,
                truth_path_xy_m=float(np.linalg.norm(np.diff(np.asarray(path_xyz)[:, :2], axis=0), axis=1).sum()) if len(path_xyz)>1 else 0.)


def evaluate(output, baseline, plans, first_stamp, origin_position, seed_covariance, args):
    # This is the first read of simulator truth, after every estimate is sealed.
    truth = load_truth(args.truth)
    stamps = [r['stamp_ns'] for r in plans]
    values, valid = interpolate_truth(truth, stamps)
    initial, initial_valid = interpolate_truth(truth, [first_stamp])
    if not bool(initial_valid[0]):
        raise ValueError('No truth bracketing the fixed initial sensor exposure')
    origin = initial[0]
    lookup = {r['stamp_ns']: r for r in baseline}
    baseline_plans = []
    paired_new, paired_base, paired_truth, paired_valid = [], [], [], []
    for row, value, usable in zip(plans, values, valid):
        old = lookup[row['stamp_ns']]
        transformed = dict(old)
        if old['output']:
            transformed['position'] = (np.asarray(old['position']) - origin_position).tolist()
        baseline_plans.append(transformed)
        if row['output'] and transformed['output']:
            paired_new.append(row)
            paired_base.append(transformed)
            paired_truth.append(value)
            paired_valid.append(usable)
    old_all_stamps = [r['stamp_ns'] for r in baseline]
    old_values, old_valid = interpolate_truth(truth, old_all_stamps)
    old_all = [dict(r, position=(np.asarray(r['position'])-origin_position).tolist()) if r['output'] else r for r in baseline]
    report = dict(
        evaluation_origin=dict(stamp_ns=first_stamp, truth_position=origin,
            baseline_position=origin_position, trajectory_alignment='initial translation only; no fitted rotation/scale/trajectory alignment'),
        planned_baseline=score(baseline_plans, values, valid, origin),
        planned_window_conditional=score(plans, values, valid, origin, 'covariance_conditional'),
        planned_window_with_start_prior=score(plans, values, valid, origin),
        common_output_baseline=score(paired_base, paired_truth, paired_valid, origin),
        common_output_window_with_start_prior=score(paired_new, paired_truth, paired_valid, origin),
        baseline_all_sensor_frames=score(old_all, old_values, old_valid, origin),
        candidate_failures=dict(Counter(r['reason'] for r in plans if not r['output'])),
        baseline_failures=dict(Counter(r['reason'] for r in baseline if not r['output'])),
        covariance_comparison=dict(start_prior_covariance=seed_covariance,
            interpretation='Candidate conditional C and C+C0 are both retained. C0 is added once only for a conservative independent-origin sensitivity comparison, not a calibrated posterior or an optimizer input.',
            shared_start_frame_cross_correlation='Not calibrated; same-image origin correlations and real AHRS temporal correlation remain limitations.'),
        timing=dict(frontend_seconds_p50_p95_max=percentiles([r['frontend_s'] for r in baseline]),
            baseline_seconds_p50_p95_max=percentiles([r['baseline_s'] for r in baseline]),
            window_solve_seconds_p50_p95_max=percentiles([r['backend_s'] for r in plans if 'backend_s' in r]),
            planned_frame_local_compute_seconds_p50_p95_max=percentiles([r['frontend_s']+r.get('backend_s', 0.) for r in plans]),
            baseline_total_compute_seconds=sum(r['baseline_s'] for r in baseline),
            candidate_all_frontend_compute_seconds=sum(r['frontend_s'] for r in baseline),
            candidate_all_backend_compute_seconds=sum(r.get('backend_s', 0.) for r in plans),
            candidate_total_compute_seconds=sum(r['frontend_s'] for r in baseline)+sum(r.get('backend_s', 0.) for r in plans),
            scope='Offline compute only; excludes archive I/O, ROS/decode/scheduling and is not an online freshness qualification.'),
        limitations=[
            'A first-order RGBD landmark/window MAP prototype, not nonlinear bundle adjustment or full VIO.',
            'Frontend seeds new measured points with ORB and maintains identities with guarded pyramidal forward/backward KLT; inter-observation LK/depth calibration correlations are not calibrated.',
            'Attitude and 3mm common measurement priors are shared by points in each exposure but independent across exposures in this prototype.',
            'Comparison is an overall 10Hz baseline vs 2Hz optimized-map output pipeline, not an isolated sliding-window ablation.',
            'No candidate covariance cutoff hides large uncertainty; all planned failures and outputs above 50mm remain in the denominator.',
            'Recorded paths were produced by the old controller; replay cannot prove what an autonomous new-controller trajectory would do.',
            'No workbench/navigation/revisit source was edited or imported by this prototype.'
        ])
    cache = getattr(args, 'baseline_cache_metadata', None)
    report['timing']['baseline_measurement_mode'] = 'cached_historical' if cache else 'current_replay'
    if cache:
        report['timing'].update(
            baseline_seconds_p50_p95_max=None, baseline_total_compute_seconds=None,
            baseline_cached_historical_seconds_p50_p95_max=cache['historical_baseline_seconds_p50_p95_max'],
            baseline_cached_historical_total_compute_seconds=cache['historical_baseline_total_compute_seconds'],
            baseline_cache=cache,
            scope=report['timing']['scope']+' Baseline baseline_s values are historical cached wall measurements, '
                  'not current-run baseline compute. All frontend_s values were measured again in this run.')
    write_json(output/'metrics.json', report)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', required=True, type=Path)
    parser.add_argument('--truth', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--interval', type=float, default=.5)
    parser.add_argument('--window', type=int, default=6)
    parser.add_argument('--baseline-run', type=Path, help='Reuse a sealed complete same-input/same-source baseline without running its estimator.')
    parser.add_argument('--max-sensor-frames', type=int, default=0, help='Explicit smoke-run truncation; 0 means complete sequence')
    args = parser.parse_args()
    if args.interval <= 0:
        parser.error('interval must be positive')
    if args.baseline_run is not None and args.max_sensor_frames:
        parser.error('--baseline-run requires a complete replay; do not truncate sensor frames')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    cv2.setNumThreads(2)
    source_files = list(Path(__file__).parent.glob('*.py')) + [SLAM_SCRIPTS/'metric_rgbd_odometry.py']
    protected_files = [Path(__file__).resolve().parents[1]/name for name in ('autonomous_survey.py', 'localization_revisit.py', 'simulation_experiment.py')]
    source_before = {str(p): digest(p) for p in source_files+protected_files}
    baseline_cache = BaselineCache(args.baseline_run, args.manifest) if args.baseline_run is not None else None
    args.baseline_cache_metadata = baseline_cache.metadata if baseline_cache is not None else None
    baseline_core = MetricRGBDOdometry() if baseline_cache is None else None
    frontend = TrackFrontend(max_frames=args.window)
    estimator = WindowEstimator(max_frames=args.window)
    baseline, plans = [], []
    initial_stamp, initial_position, seed_C, next_due = None, None, None, None
    interval_ns = int(round(args.interval*1e9))
    started = time.perf_counter()
    raw_path, plans_path = output/'baseline.jsonl', output/'window.jsonl'
    with raw_path.open('x', encoding='utf-8') as raw_file, plans_path.open('x', encoding='utf-8') as plan_file:
        for index, sensor in enumerate(iter_sensor_frames(args.manifest, verify_hashes=True)):
            if args.max_sensor_frames and index >= args.max_sensor_frames:
                break
            if baseline_cache is not None:
                old_row = baseline_cache.row_for(sensor.stamp_ns)
            else:
                t = time.perf_counter()
                old = baseline_core.update(sensor.stamp, sensor.gray, sensor.xyz_grid, sensor.yaw,
                                           sensor.wheel_vx, sensor.wheel_wz, camera_model=sensor.camera_model)
                old_elapsed = time.perf_counter()-t
                old_row = dict(stamp_ns=sensor.stamp_ns, output=bool(old['accepted']), reason=old['reason'], baseline_s=old_elapsed)
                if old['accepted']:
                    old_row.update(position=old['xyz'], covariance=old['covariance'], reference_id=old.get('reference_id'))
            if initial_stamp is None:
                if not old_row['output']:
                    raise ValueError('Baseline cannot initialize on the fixed first input exposure')
                initial_stamp = sensor.stamp_ns
                initial_position = np.asarray(old_row['position']).copy()
                seed_C = np.asarray(old_row['covariance']).copy()
                next_due = initial_stamp
            t = time.perf_counter()
            features = frontend.process(sensor)
            frontend_elapsed = time.perf_counter()-t
            old_row['frontend_s'] = frontend_elapsed
            baseline.append(plain(old_row))
            raw_file.write(json.dumps(plain(old_row), allow_nan=False)+'\n')
            if sensor.stamp_ns < next_due:
                continue
            scheduled = next_due
            while next_due <= sensor.stamp_ns:
                next_due += interval_ns
            row = dict(stamp_ns=sensor.stamp_ns, planned_stamp_ns=scheduled, output=False,
                       reason='no_features', frontend_s=frontend_elapsed)
            if features is not None:
                selected, diagnostic = frontend.select(features)
                row['frontend'] = diagnostic
                if not diagnostic['adequate']:
                    row['reason'] = 'insufficient_shared_or_spatial_track_support'
                else:
                    R = sensor.nominal_R_wb @ sensor.R_bc
                    lever = sensor.nominal_R_wb @ sensor.t_bc
                    observations = []
                    for point_index in selected:
                        # Undo only the deterministic nominal coordinate change.
                        # These are the same raw measured points, not a second
                        # independent observation of the IMU-rotated grid.
                        optical = R.T @ (features['points'][point_index]-lever)
                        optical_C = R.T @ features['covariance'][point_index] @ R
                        observations.append(Observation(int(features['track_ids'][point_index]), optical, optical_C))
                    frame = Frame(sensor.stamp_ns, sensor.nominal_R_wb, sensor.R_bc, sensor.t_bc,
                                  observations, np.deg2rad(.1)**2*np.eye(3))
                    t = time.perf_counter()
                    try:
                        result = estimator.add_frame(frame)
                        row.update(output=True, reason='solved', position=result.position,
                            covariance_conditional=result.covariance, covariance=result.covariance+seed_C,
                            diagnostics=result.diagnostics)
                        frontend.commit(features, selected)
                    except WindowError as error:
                        row['reason'] = 'window_error: '+str(error)
                    row['backend_s'] = time.perf_counter()-t
            if not plans and not row['output']:
                raise ValueError('Candidate cannot initialize on the same fixed first exposure: '+row['reason'])
            plans.append(plain(row))
            plan_file.write(json.dumps(plain(row), allow_nan=False)+'\n')
            raw_file.flush()
            plan_file.flush()
            if len(plans) % 20 == 0:
                print(json.dumps(dict(sensor_frames=len(baseline), planned=len(plans), outputs=sum(r['output'] for r in plans), stamp=sensor.stamp,
                    last_reason=row['reason'], elapsed_s=round(time.perf_counter()-started, 2))), flush=True)
    if baseline_core is not None:
        baseline_core.close()
    if baseline_cache is not None:
        baseline_cache.finish()
    metrics = evaluate(output, baseline, plans, initial_stamp, initial_position, seed_C, args)
    source_after = {str(p): digest(p) for p in source_before}
    if source_before != source_after:
        raise RuntimeError('Bound source changed during replay; do not treat run as frozen')
    write_json(output/'run.json', dict(config=vars(args), elapsed_s=time.perf_counter()-started,
        sensor_frames=len(baseline), planned_frames=len(plans), frontend_stats=frontend.stats,
        baseline_cache=baseline_cache.metadata if baseline_cache is not None else None,
        source_before=source_before, source_after=source_after, manifest_sha256=digest(args.manifest), truth_sha256=digest(args.truth),
        output_sha256={p.name: digest(p) for p in (raw_path, plans_path, output/'metrics.json')},
        runtime=dict(python=sys.version, numpy=np.__version__, opencv=cv2.__version__,
            env={name: os.environ.get(name) for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS')})))
    print(json.dumps(plain(dict(complete=True, frames=len(baseline), planned=len(plans), outputs=metrics['planned_window_with_start_prior']['outputs'],
        baseline=metrics['planned_baseline'], window=metrics['planned_window_with_start_prior'], timing=metrics['timing'])), allow_nan=False), flush=True)


if __name__ == '__main__':
    main()
