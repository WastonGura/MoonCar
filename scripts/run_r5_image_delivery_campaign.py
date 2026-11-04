#!/usr/bin/env python3
"""Eight bounded delivery diagnostics, then four independent calibrations."""
import argparse
import json
from pathlib import Path
from r5_image_delivery import delivery_arguments,choose_candidate
from run_r5_rgb_gap_campaign import run as round_run
from run_r5_d0_acceptance import CampaignStop

def stage_allowed(entries,stage,profile):
    labels=[e['name'].split('-')[1] for e in entries if e['stage']=='diagnostic']
    if stage=='baseline' and entries:raise CampaignStop('Fresh baseline directory required')
    if stage=='repair' and (len(labels)!=4 or labels.count('stock')!=2 or labels.count('traced')!=2):
        raise CampaignStop('Four baseline observations required')
    if stage=='restore' and (len(labels)!=6 or labels.count('repair')!=2):raise CampaignStop('Two repair rounds required')
    if stage=='calibrate':
        repair=[e for e in entries if '-repair-' in e['name']]
        traced=[e for e in entries if '-traced-' in e['name']]
        no_reproduction=len(entries)==4 and all(e.get('passed') for e in entries) and len(traced)==2
        verified=len(entries)==8 and len(repair)==2 and all(e.get('passed') for e in repair)
        if not(no_reproduction or verified):raise CampaignStop('Missing qualified repair/restore matrix or four un-reproduced complete baselines')
        if no_reproduction and profile!='legacy':raise CampaignStop('Un-reproduced calibration must retain legacy delivery')

def run(args):
    root=Path(args.output);root.mkdir(parents=True,exist_ok=True)
    ledger=root/'ledger.json';entries=json.loads(ledger.read_text()) if ledger.exists() else []
    if (root/'campaign-stop.json').exists():raise CampaignStop('Stopped campaign cannot resume')
    stage_allowed(entries,args.stage,args.image_delivery_profile)
    args.calibrate=False;args.image_sampling='legacy';args.image_trace='headers'
    if args.stage=='baseline':
        for bridge in ('stock','traced'):
            args.image_bridge=bridge;args.image_delivery_profile='legacy'
            for number in (1,2):args.label=f'{bridge}-{number}';round_run(args)
        return 0
    if args.stage=='repair':
        if not args.candidate_evidence:raise CampaignStop('Candidate correlation evidence required')
        evidence=json.loads(Path(args.candidate_evidence).read_text())
        candidate=choose_candidate(evidence)
        if candidate!=args.image_delivery_profile or not candidate:raise CampaignStop('No single evidence-qualified repair')
        args.image_bridge='traced'
    if args.stage=='restore':args.image_bridge='traced';args.image_delivery_profile='legacy'
    if args.stage in ('repair','restore'):
        for number in (1,2):args.label=f'{args.stage}-{number}';round_run(args)
        return 0
    args.image_bridge='traced';args.calibrate=True
    return round_run(args)

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);delivery_arguments(p)
    for n in ('output','model','plugin','observer-plugin','image-observer'):p.add_argument('--'+n,required=True)
    p.add_argument('--package',default='/source/ros2_ws/src/mooncar_gazebo')
    p.add_argument('--stage',choices=('baseline','repair','restore','calibrate'),required=True)
    p.add_argument('--candidate-evidence')
    raise SystemExit(run(p.parse_args()))
