"""Bounded, closed-video cached-query comparison. Never starts ROS or a world.

The frontend is frozen once. Reference and candidate own independent persistent
windows. Every imported measurement and planned failure stays in the denominator.
Truth is opened only after equivalence and timing estimates have been sealed.
"""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import pickle
import sys
import time

import cv2
import numpy as np

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from offline_rgbd_window.core import Frame, Observation, WindowEstimator, WindowError
from offline_rgbd_window.data import load_truth, interpolate_truth
from offline_rgbd_window.frontend import TrackFrontend
from offline_rgbd_window.online import PlanScheduler, plain
from offline_rgbd_window.query_cache import freeze_query_cache, query_cached

ROOT = Path(__file__).resolve().parents[2]
SCORE_NS = 31_346_000_000
END_NS = 40_500_000_000
START_NS = 28_000_000_000
P_TOL = dict(atol=2e-8, rtol=1e-7)
C_TOL = dict(atol=2e-11, rtol=1e-6)
BOUND = ('scripts/offline_rgbd_window/core.py', 'scripts/offline_rgbd_window/frontend.py',
         'scripts/offline_rgbd_window/online.py', 'scripts/offline_rgbd_window/data.py',
         'scripts/offline_rgbd_window/query_cache.py',
         'scripts/offline_rgbd_window/compare_cached_query.py',
         'scripts/offline_rgbd_window/extract_query_clip.py')


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            h.update(block)
    return h.hexdigest()


def save(path, value):
    with path.open('x', encoding='utf-8') as stream:
        json.dump(plain(value), stream, indent=2, allow_nan=False)
        stream.write('\n')


def stats(values):
    values = np.asarray(list(values), dtype=float)
    return dict(count=len(values), p50=float(np.median(values)) if len(values) else None,
                p95=float(np.percentile(values, 95)) if len(values) else None,
                maximum=float(values.max()) if len(values) else None,
                total=float(values.sum()))


def safe_counterexample(value):
    value = plain(value)
    if isinstance(value, dict):
        return {k:safe_counterexample(v) for k,v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_counterexample(v) for v in value]
    if isinstance(value, float) and not np.isfinite(value):
        return str(value)
    return value


def timed(call):
    wall, cpu = time.perf_counter(), time.process_time()
    try:
        result = call()
        status = dict(output=True, reason='solved', position=result.position,
                      covariance=result.covariance, diagnostics=result.diagnostics)
    except WindowError as error:
        status = dict(output=False, reason=str(error))
    return status, dict(wall_s=time.perf_counter()-wall, cpu_s=time.process_time()-cpu)


def freeze_frontend(sensors, output, guard):
    frontend, estimator = TrackFrontend(96, 6), WindowEstimator(6)
    scheduler, last_keyframe = PlanScheduler(100_000_000), None
    records, trace, gauge = [], [], None
    for sensor in sensors:
        guard()
        planned, missing = scheduler.observe(sensor.stamp_ns)
        if missing or planned is None:
            raise ValueError('Fixed clip must have one real paired exposure per declared slot')
        wall, cpu = time.perf_counter(), time.process_time()
        features = frontend.process(sensor)
        diagnostic, selected = {}, None
        if features is not None:
            selected, diagnostic = frontend.select(features)
        front = dict(wall_s=time.perf_counter()-wall, cpu_s=time.process_time()-cpu)
        keyframe = last_keyframe is None or sensor.stamp_ns-last_keyframe >= 500_000_000
        record = dict(stamp_ns=sensor.stamp_ns, planned_stamp_ns=planned,
                      keyframe=keyframe, frame=None, frontend=diagnostic, frontend_timing=front)
        if features is None:
            status = dict(output=False, reason='no_features')
        elif not diagnostic['adequate']:
            status = dict(output=False, reason='insufficient_shared_or_spatial_track_support')
        else:
            rotation = sensor.nominal_R_wb @ sensor.R_bc
            lever = sensor.nominal_R_wb @ sensor.t_bc
            observations = [Observation(int(features['track_ids'][i]),
                rotation.T @ (features['points'][i]-lever),
                rotation.T @ features['covariance'][i] @ rotation) for i in selected]
            frame = Frame(sensor.stamp_ns, sensor.nominal_R_wb.copy(), sensor.R_bc.copy(),
                sensor.t_bc.copy(), observations, np.deg2rad(.1)**2*np.eye(3))
            record['frame'] = frame
            status, _ = timed(lambda: estimator.add_frame(frame) if keyframe else estimator.query_frame(frame))
            if status['output']:
                if gauge is None:
                    gauge = sensor.stamp_ns
                if keyframe:
                    frontend.commit(features, selected)
                    last_keyframe = sensor.stamp_ns
        record['expected'] = status
        record['persistent_keyframes'] = len(estimator._frames)
        records.append(record)
        trace.append({k:v for k,v in record.items() if k != 'frame'})
    score = [r for r in records if r['stamp_ns'] >= SCORE_NS]
    warm = [r for r in records if r['stamp_ns'] < SCORE_NS]
    if not warm or warm[-1]['persistent_keyframes'] != 6:
        raise ValueError('data_insufficient: pre-score window did not warm to six actual keyframes')
    if not any(r['frame'] is not None and not r['keyframe'] for r in score):
        raise ValueError('data_insufficient: no scored ordinary query')
    if not any(r['frame'] is not None and r['keyframe'] for r in score):
        raise ValueError('data_insufficient: no scored keyframe boundary')
    with (output/'frozen-inputs.pickle').open('xb') as stream:
        pickle.dump(dict(records=records, gauge_stamp_ns=gauge), stream, protocol=5)
    with (output/'frontend.jsonl').open('x') as stream:
        for row in trace:
            stream.write(json.dumps(plain(row), allow_nan=False)+'\n')
    return records, gauge


def replay(records, candidate, guard, check_state=False, mismatch_path=None, label=None):
    estimator, cache, version = WindowEstimator(6), None, 0
    input_fingerprint = hashlib.sha256(pickle.dumps(records,protocol=5)).hexdigest()
    rows = []
    for record in records:
        guard()
        frame = record['frame']
        kind = 'keyframe' if record['keyframe'] else 'query'
        before = pickle.dumps(estimator.__dict__) if check_state and kind == 'query' else None
        if frame is None:
            status = copy.deepcopy(record['expected'])
            elapsed = dict(wall_s=0., cpu_s=0.)
        else:
            status, elapsed = timed(lambda: estimator.add_frame(frame) if kind == 'keyframe'
                else query_cached(frame, cache, version) if candidate else estimator.query_frame(frame))
        cache_elapsed = dict(wall_s=0., cpu_s=0.)
        if candidate and frame is not None and kind == 'keyframe' and status['output']:
            version += 1
            persistent = pickle.dumps(estimator.__dict__) if check_state else None
            wall, cpu = time.perf_counter(), time.process_time()
            cache = freeze_query_cache(estimator, version)
            cache_elapsed = dict(wall_s=time.perf_counter()-wall, cpu_s=time.process_time()-cpu)
            if check_state and pickle.dumps(estimator.__dict__) != persistent:
                raise AssertionError('cache construction mutated persistent window')
        if check_state and before is not None and pickle.dumps(estimator.__dict__) != before:
            raise AssertionError('query mutated persistent window')
        rows.append(dict(stamp_ns=record['stamp_ns'], planned_stamp_ns=record['planned_stamp_ns'],
            kind=kind, eligible=frame is not None, status=status, timing=elapsed,
            cache_timing=cache_elapsed, cache_version=version if candidate else None,
            cache_bytes=cache.estimated_bytes() if candidate and cache is not None else 0))
        okay, difference = equivalent(record['expected'],status)
        if not okay:
            if mismatch_path is not None and not mismatch_path.exists():
                save(mismatch_path,safe_counterexample(dict(label=label,arm='candidate' if candidate else 'reference',
                    record={k:v for k,v in record.items() if k!='frame'},row=rows[-1],difference=difference)))
            raise AssertionError('numerical_or_status_mismatch at '+str(record['stamp_ns']))
        if check_state and hashlib.sha256(pickle.dumps(records,protocol=5)).hexdigest()!=input_fingerprint:
            raise AssertionError('query mutated frozen frame inputs')
    if hashlib.sha256(pickle.dumps(records,protocol=5)).hexdigest()!=input_fingerprint:
        raise AssertionError('replay mutated frozen inputs')
    return rows


def equivalent(reference, candidate):
    if reference['output'] != candidate['output']:
        return False, dict(reason='output_status_mismatch')
    if not reference['output']:
        return reference['reason'] == candidate['reason'], dict(reason='failure_reason_comparison')
    p = np.asarray(reference['position'])-candidate['position']
    C = np.asarray(reference['covariance'])-candidate['covariance']
    okay = (np.allclose(reference['position'], candidate['position'], **P_TOL)
            and np.allclose(reference['covariance'], candidate['covariance'], **C_TOL))
    return okay, dict(position_max_abs_m=float(np.max(np.abs(p))),
                      covariance_max_abs_m2=float(np.max(np.abs(C))))


def summarize_timing(rows):
    selected = [r for r in rows if r['stamp_ns'] >= SCORE_NS]
    result = {}
    for kind in ('query', 'keyframe'):
        part = [r for r in selected if r['kind'] == kind and r['eligible']]
        result[kind] = {unit: stats(r['timing'][unit] for r in part) for unit in ('wall_s','cpu_s')}
    result['cache_build'] = {unit: stats(r['cache_timing'][unit] for r in selected
        if r['kind'] == 'keyframe' and r['eligible'] and r['status']['output']) for unit in ('wall_s','cpu_s')}
    result['total_backend'] = {unit: stats(r['timing'][unit]+r['cache_timing'][unit]
        for r in selected) for unit in ('wall_s','cpu_s')}
    return result


def truth_metrics(rows, truth_path, gauge):
    truth = load_truth(truth_path)
    origin, valid = interpolate_truth(truth, [gauge])
    if not valid[0]:
        raise ValueError('No legal truth bracket at local gauge')
    scored = [r for r in rows if r['stamp_ns'] >= SCORE_NS]
    metrics = {}
    for name, left, right in (('whole_scored', SCORE_NS, END_NS),
            ('moving_action',31_346_000_000,36_554_000_000),
            ('turning_action',37_686_000_000,39_904_000_000)):
        part = [r for r in scored if left <= r['stamp_ns'] <= right]
        outputs = [r for r in part if r['status']['output']]
        xyz, okay = interpolate_truth(truth, [r['stamp_ns'] for r in outputs])
        errors, sigmas = [], []
        for r, pos, valid in zip(outputs, xyz, okay):
            status = r['status']
            C = np.asarray(status['covariance'])
            sigmas.append(float(np.sqrt(np.linalg.eigvalsh(C[:2,:2]).max())))
            if valid:
                errors.append(float(np.linalg.norm((np.asarray(status['position'])-(pos-origin[0]))[:2])))
        metrics[name] = dict(plans=len(part), outputs=len(outputs), truth_scored=len(errors),
            error_xy_m=stats(errors), conditional_sigma_xy_m=stats(sigmas),
            first_last_conditional_sigma_xy_m=[sigmas[0],sigmas[-1]] if sigmas else [])
    return metrics


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--truth', type=Path, required=True, help='Evaluation only, opened after seal')
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; evidence will not be overwritten')
    args.output.mkdir(parents=True)
    cv2.setNumThreads(2)
    started, cpu_started = time.perf_counter(), time.process_time()
    source_before = {name:sha(ROOT/name) for name in BOUND}
    summary = dict(outcome='incomplete', source_before=source_before,
        position_tolerance=P_TOL, covariance_tolerance=C_TOL,
        scope='Fixed local closed-video A/B only; no online latency, continuity or navigation qualification',
        clip_ns=[START_NS,END_NS], score_start_ns=SCORE_NS,
        settings=dict(blas_threads=os.environ.get('OPENBLAS_NUM_THREADS'),
            omp_threads=os.environ.get('OMP_NUM_THREADS'), opencv_threads=cv2.getNumThreads(),
            affinity=sorted(os.sched_getaffinity(0)), nice=os.nice(0)),
        acceptance=dict(query_wall_p95_reduction_at_least=.30, total_backend_wall_reduction_at_least=.20))

    def guard():
        if time.process_time()-cpu_started > 300 or time.perf_counter()-started > 360:
            raise TimeoutError('bounded offline comparison CPU300/wall360 second limit')

    try:
        manifest = json.loads((args.input/'manifest.json').read_text())
        if not manifest.get('data_sufficient') or not manifest.get('complete'):
            raise ValueError('data_insufficient: strict clip extraction failed')
        if sha(args.input/'sensors.pickle')!=manifest['sensors_sha256']:
            raise ValueError('data_insufficient: sensor pickle differs from sealed manifest')
        if manifest['clip_ns'] != [START_NS,END_NS] or manifest['score_start_ns'] != SCORE_NS:
            raise ValueError('data_insufficient: fixed clip boundaries changed')
        summary['input_sha256'] = {p.name:sha(p) for p in args.input.iterdir() if p.is_file()}
        with (args.input/'sensors.pickle').open('rb') as stream:
            sensors = pickle.load(stream)
        assert [s.stamp_ns for s in sensors] == list(range(START_NS,END_NS+1,100_000_000))
        records, gauge = freeze_frontend(sensors,args.output,guard)
        del sensors
        print(json.dumps(dict(stage='frontend_frozen',frames=len(records),gauge_stamp_ns=gauge)),flush=True)
        mismatch_path=args.output/'first-mismatch.json'
        candidate = replay(records,True,guard,check_state=True,mismatch_path=mismatch_path,label='equivalence')
        differences = []
        for record, row in zip(records,candidate):
            okay, difference = equivalent(record['expected'],row['status'])
            differences.append(dict(stamp_ns=record['stamp_ns'],**difference))
            if not okay:
                save(args.output/'first-mismatch.json',dict(
                    record={k:v for k,v in record.items() if k!='frame'},row=row,difference=difference))
                raise AssertionError('numerical_or_status_mismatch at '+str(record['stamp_ns']))
        save(args.output/'equivalence.json',dict(passed=True,plans=len(records),rows=differences,
            maximum_position_abs_m=max((r.get('position_max_abs_m',0) for r in differences),default=0),
            maximum_covariance_abs_m2=max((r.get('covariance_max_abs_m2',0) for r in differences),default=0),
            persistent_window_unchanged_by_queries_and_cache=True))
        print(json.dumps(dict(stage='equivalence_passed',plans=len(records))),flush=True)
        # Import/allocator warmup only. Each replay still rebuilds all model caches.
        replay(records,False,guard,mismatch_path=mismatch_path,label='warmup')
        replay(records,True,guard,mismatch_path=mismatch_path,label='warmup')
        timings, outputs = [], {}
        with (args.output/'timing-plans.jsonl').open('x') as stream:
            for repeat, order in enumerate(((False,True),(True,False),(False,True)),1):
                pair = {}
                for is_candidate in order:
                    arm = 'candidate' if is_candidate else 'reference'
                    rows = replay(records,is_candidate,guard,mismatch_path=mismatch_path,label='timing_'+str(repeat))
                    for record,row in zip(records,rows):
                        okay,_ = equivalent(record['expected'],row['status'])
                        if not okay:
                            raise AssertionError('Timed repeat changed numerical or failure outcome')
                        stream.write(json.dumps(plain(dict(repeat=repeat,arm=arm,**row)),allow_nan=False)+'\n')
                    pair[arm] = summarize_timing(rows)
                    outputs[arm] = rows
                timings.append(dict(repeat=repeat,order=['candidate' if v else 'reference' for v in order],arms=pair))
                print(json.dumps(dict(stage='timing_pair_complete',repeat=repeat)),flush=True)
        save(args.output/'timing.json',timings)
        query_reductions = [1-r['arms']['candidate']['query']['wall_s']['p95']/r['arms']['reference']['query']['wall_s']['p95'] for r in timings]
        total_reductions = [1-r['arms']['candidate']['total_backend']['wall_s']['total']/r['arms']['reference']['total_backend']['wall_s']['total'] for r in timings]
        summary.update(equivalence_passed=True,plans=len(records),
            query_wall_p95_reduction_by_pair=query_reductions,
            total_backend_wall_reduction_by_pair=total_reductions,
            query_wall_p95_reduction_median=float(np.median(query_reductions)),
            total_backend_wall_reduction_median=float(np.median(total_reductions)),
            frontend_scored_timing={unit:stats(r['frontend_timing'][unit] for r in records if r['stamp_ns']>=SCORE_NS) for unit in ('wall_s','cpu_s')})
        summary['maximum_candidate_cache_bytes'] = max(r['cache_bytes'] for r in outputs['candidate'])
        passed = summary['query_wall_p95_reduction_median'] >= .30 and summary['total_backend_wall_reduction_median'] >= .20
        summary['outcome'] = 'equivalent_with_benefit' if passed else 'correct_but_insufficient_benefit'
        with (args.output/'sealed-estimates.jsonl').open('x') as stream:
            for arm, rows in outputs.items():
                for row in rows:
                    stream.write(json.dumps(plain(dict(arm=arm,**row)),allow_nan=False)+'\n')
        save(args.output/'estimator-seal.json',dict(
            files={name:sha(args.output/name) for name in ('sealed-estimates.jsonl','timing-plans.jsonl','frozen-inputs.pickle','equivalence.json','timing.json')},
            truth_not_opened_by_estimation=True))
        summary['evaluation'] = {arm:truth_metrics(rows,args.truth,gauge) for arm,rows in outputs.items()}
        summary['evaluation_truth_sha256'] = sha(args.truth)
        summary['coordinate_contract'] = 'Original declared yaw=0 offset=0 axes; translation at local first successful keyframe; no rotation/scale fit; covariance conditional'
    except Exception as error:
        summary.update(outcome='numerical_non_equivalence' if isinstance(error,AssertionError)
                       else 'data_insufficient' if 'data_insufficient' in str(error) else 'bounded_failure',
                       error=dict(type=type(error).__name__,message=str(error)))
    finally:
        summary['source_after'] = {name:sha(ROOT/name) for name in BOUND}
        summary['source_unchanged'] = summary['source_before']==summary['source_after']
        if not summary['source_unchanged']:
            summary['outcome'] = 'bounded_failure'
            summary['error'] = dict(type='SourceChanged',message='A bound input source changed during comparison')
        summary['elapsed_wall_s'] = time.perf_counter()-started
        summary['process_cpu_s'] = time.process_time()-cpu_started
        save(args.output/'summary.json',summary)
    print(json.dumps(dict(outcome=summary['outcome'],output=str(args.output),
        elapsed_wall_s=summary['elapsed_wall_s'],error=summary.get('error'))),flush=True)
    return 0 if summary['outcome'] in ('equivalent_with_benefit','correct_but_insufficient_benefit') else 1


if __name__ == '__main__':
    raise SystemExit(main())
