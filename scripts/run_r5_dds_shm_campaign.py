#!/usr/bin/env python3
"""Fresh bounded stock/observed/8MiB/restore matrix, then four calibrations."""
import argparse,json,hashlib
from pathlib import Path
from r5_image_delivery import delivery_arguments
from r5_dds_shm import qualify_candidate,diagnostic_complete,project_runtime
from run_r5_rgb_gap_campaign import run as round_run
from run_r5_d0_acceptance import CampaignStop,SOURCES

def progress(entries,causal=False,repair=False,reason=''):
    completed=[e for e in entries if e.get('status')!='started']
    return dict(status='stopped',reason=reason,attempts=len(entries),completed_attempts=len(completed),
        causal_confirmed=causal,repair_verified=repair,
        calibration_rounds=sum(e.get('stage')=='calibration' for e in entries),
        calibration_passed=sum(e.get('stage')=='calibration' and bool(e.get('passed')) for e in entries),
        publication=False,vehicle_acceptance=False)

def tool_signature(args):
    digest=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
    observer=Path('/source/containers/r5_dds_observer')
    resources=json.loads(Path('/source/reports/evidence/terrain-v2/r5-capture-repair-20261007/source-and-resource-hashes.json').read_text())['resources']
    return dict(sources={n:digest(Path(__file__).with_name(n)) for n in SOURCES},
        project_runtime=project_runtime(),
        observer_sources={p.name:digest(p) for p in observer.iterdir() if p.is_file()},
        resources={n:digest(next(p for p in (Path('/source')/n,Path(args.package)/n) if p.is_file())) for n in resources},
        model_urdf=digest(Path(args.model).with_suffix('.urdf')),
        artifacts={n:digest(getattr(args,n)) for n in ('model','plugin','observer_plugin','image_observer','traced_bridge','cpp_image_observer')},
        runtime_manifest=digest(Path(args.dds_prefix)/'manifest.json'))

def run(args):
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    if (root/'ledger.json').exists() or (root/'campaign-stop.json').exists():
        raise CampaignStop('Fresh campaign directory required; never resume stopped evidence')
    preflight=json.loads(Path(args.observer_preflight).read_text())
    digest=lambda path:hashlib.sha256(Path(path).read_bytes()).hexdigest()
    if not preflight.get('passed') or preflight.get('project_runtime')!=project_runtime() or preflight.get('runtime_manifest_sha256')!=digest(Path(args.dds_prefix)/'manifest.json') or \
            preflight.get('preflight_source_sha256')!=digest(Path(__file__).with_name('preflight_r5_dds_observer.py')) or \
            preflight.get('test_source_sha256')!=digest('/source/ros2_ws/src/mooncar_gazebo/tests/test_dds_shm.py'):
        raise CampaignStop('Observer preflight missing, failed or belongs to another signature')
    args.calibrate=False;args.image_sampling='legacy';args.image_trace='headers';args.image_bridge='traced'
    frozen=tool_signature(args)
    (root/'observer-freeze.json').write_text(json.dumps(frozen,indent=2)+'\n')
    def entries():return json.loads((root/'ledger.json').read_text()) if (root/'ledger.json').exists() else []
    def results(selected):return [json.loads((root/e['name']/'result.json').read_text()) for e in selected]
    def unchanged():
        if tool_signature(args)!=frozen:raise CampaignStop('Execution/observation signature changed after first-run freeze')
    def stage(name,mode,profile):
        args.dds_runtime=mode;args.dds_trace='images' if mode=='observed' else 'off';args.image_delivery_profile=profile
        for n in (1,2):
            unchanged();args.label=f'{name}-{n}';round_run(args);unchanged()
            e=entries()[-1]
            if e.get('status')=='wall_timeout':raise CampaignStop('Whole-round wall timeout')
            diagnostic_complete(results([e])[0],mode=='observed')
    causal=False;repair=False
    try:
        stage('stockdds','stock','legacy')
        stage('observeddds','observed','legacy')
        qualify_candidate(entries(),results(entries()[2:4]))
        proof=dict(runtime_manifest_sha256=hashlib.sha256((Path(args.dds_prefix)/'manifest.json').read_bytes()).hexdigest(),
            observed_defaults=[dict(result=str(root/e['name']/'result.json'),
                sha256=hashlib.sha256((root/e['name']/'result.json').read_bytes()).hexdigest()) for e in entries()[2:4]])
        args.dds_candidate_evidence=str(root/'candidate-evidence.json')
        Path(args.dds_candidate_evidence).write_text(json.dumps(proof,indent=2)+'\n')
        stage('repair','observed','shm_tx8m')
        repairs=results(entries()[4:6])
        if not all(e['passed'] for e in entries()[4:6]) or any(r.get('dds',{}).get('capacity_links') for r in repairs):
            raise CampaignStop('8MiB candidate not complete or identified capacity loss remains')
        repair=True
        stage('restore','observed','legacy')
        restores=results(entries()[6:8])
        causal=all(not r.get('passed') and r['dds'].get('capacity_qualified') for r in restores)
        (root/'diagnosis.json').write_text(json.dumps(dict(causal_confirmed=causal,repair_verified=repair,
            reason='Same capacity/loss mechanism restored twice' if causal else 'Restoration did not confirm cause'),indent=2)+'\n')
        args.calibrate=True;args.image_delivery_profile='shm_tx8m';unchanged()
        code=round_run(args);unchanged()
        if code:raise CampaignStop('Frozen calibration failed')
        done=progress(entries(),causal,repair,'Four fresh calibrations and command pairs passed');done['status']='passed'
        (root/'campaign-final.json').write_text(json.dumps(done,indent=2)+'\n')
        return 0
    except (RuntimeError,KeyError,FileNotFoundError) as e:
        data=progress(entries(),causal,repair,str(e))
        (root/'campaign-stop.json').write_text(json.dumps(data,indent=2)+'\n')
        print(json.dumps(data),flush=True);return 1

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);delivery_arguments(p)
    p.add_argument('--observer-preflight',required=True)
    for n in ('output','model','plugin','observer-plugin','image-observer'):p.add_argument('--'+n,required=True)
    p.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    raise SystemExit(run(p.parse_args()))
