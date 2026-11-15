"""Summarize the paired recorded missions using the existing survey metrics."""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
from survey_evaluation import metrics


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def rows(path):
    with path.open(encoding='utf-8') as stream:
        return [json.loads(s) for s in stream if s.strip()]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    reports=[]
    for name in ('baseline','candidate'):
        folder=args.raw/name
        result=read(folder/'case/result.json')
        survey=result.get('autonomy',{})
        start=survey.get('started')
        stop=start+survey.get('elapsed_seconds',0) if start is not None else None
        experiment=result.get('experiment')
        recording=read(folder/'video/recording.json')
        software=read(args.raw/(name+'-sessions')/experiment/'software.json')
        history=survey.get('history',[])
        truth=rows(folder/'case/observer/truth.jsonl')
        alignment=rows(folder/'case/observer/alignment.jsonl')
        truth=[r for r in truth if start is not None and start<=r['stamp']<=stop]
        alignment=[r for r in alignment if start is not None and start<=r['stamp']<=stop]
        track=[]
        last=-math.inf
        for r in truth:
            if r['stamp']-last>=.1:
                track.append(r['xyz'][:2])
                last=r['stamp']
        distance=sum(math.dist(a,b) for a,b in zip(track,track[1:]))
        errors=[r['xy_error_m'] for r in alignment]
        research=args.raw/(name+'-sessions')/experiment/'research'
        quality=list(research.glob('autonomy-*/quality-end.json'))
        quality_result=metrics(read(quality[0]),survey['origin'],survey['radius']) if quality and survey.get('origin') else None
        summary=dict(strategy=name,experiment=experiment,terrain_strategy=software.get('terrain_strategy'),
                     tested_commit=result['software']['git_commit'],outcome=result['outcome'],
                     phase=survey.get('phase'),reason=survey.get('reason'),budget_seconds=survey.get('budget_seconds'),
                     elapsed_seconds=survey.get('elapsed_seconds'),wall_seconds=result.get('elapsed_wall_s'),
                     chosen_actions=len(history),successful_actions=sum(bool(r.get('success')) for r in history),
                     movement_actions=sum(r.get('kind')!='turn' for r in history),
                     localization_pauses=len(survey.get('localization_pauses',[])),
                     truth_track_length_m_sampled_10hz=distance,coverage=quality_result,
                     xy_error_m=dict(median=float(np.median(errors)),p95=float(np.percentile(errors,95)),
                                     maximum=max(errors)) if errors else None,
                     actual_gui_captures=recording['frame_count'],capture_error=recording.get('error'),
                     encoder_exit=recording['encoder_exit'],video_process_exit=read(folder/'video-exit.json')['exit_code'],
                     driver_exit=read(folder/'driver-exit.json')['exit_code'],
                     backend_exit=read(folder/'backend-exit.json')['exit_code'],
                     camera_follow_confirmed=read(folder/'case/camera/ready.json')['confirmed'])
        reports.append(summary)
    args.output.write_text(json.dumps(reports,ensure_ascii=False,indent=2)+'\n',encoding='utf-8',newline='\n')
    print(json.dumps(reports,ensure_ascii=False),flush=True)


if __name__=='__main__':
    main()
