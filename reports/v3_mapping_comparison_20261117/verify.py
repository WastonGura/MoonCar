"""Audit map receipts against displayed footage and hash the retained evidence."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


OUTPUT = Path(__file__).resolve().parent


def read(path):
    return json.loads(path.read_text(encoding='utf-8'))


def sha(path):
    value = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            value.update(chunk)
    return value.hexdigest()


def main():
    global OUTPUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--output',type=Path,default=OUTPUT)
    parser.add_argument('--budget',type=float,default=180.)
    args = parser.parse_args()
    OUTPUT=args.output.resolve()
    paths = []
    sources = []
    pose_sources = []
    capacity_warnings=[]
    for name in ('baseline', 'candidate'):
        folder = args.raw/name
        slim = []
        with (folder/'case/observer/survey-map.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                slim.append(dict(wall=row['wall'], sim=row['sim'], stamp=row['payload']['stamp']))
        sources.append(slim)
        poses = []
        with (folder/'case/states.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                row = json.loads(line)
                if row['state']['motion'].get('pose'):
                    poses.append(dict(wall=row['wall'], sim=row['state']['motion'].get('diagnostics', {}).get('sim_stamp')))
        pose_sources.append(poses)
        warning=None
        with (folder/'case/observer/samples.jsonl').open(encoding='utf-8') as stream:
            for line in stream:
                event=json.loads(line)
                if event.get('topic')=='/mooncar/map/status' and any('Session capacity reached' in s.get('message','') for s in event.get('status',[])):
                    warning=event['wall'];break
        capacity_warnings.append(warning)
        paths += [folder/p for p in ('case/observer/survey-map.jsonl', 'case/result.json',
                                     'case/protocol.json', 'protocol.json', 'case/observer/status.json','case/resources.jsonl')]
        result = read(folder/'case/result.json')
        research = args.raw/(name+'-sessions')/result['experiment']/'research'
        paths += list(research.glob('autonomy-*/quality-*.json'))
        paths += list(research.glob('autonomy-*/telemetry.jsonl'))
        paths += [args.raw/(name+'-sessions')/result['experiment']/'scene-setup.json',
                  args.raw/(name+'-sessions')/result['experiment']/'task.json',
                  args.raw/(name+'-sessions')/result['experiment']/'software.json']
        paths += list((args.raw/(name+'-sessions')/result['experiment']).glob('survey-*.npz'))
    checked = 0
    with (OUTPUT/'overlay-timing.jsonl').open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            for side, record in enumerate(row['sides']):
                warning=capacity_warnings[side]
                expected=warning if warning is not None and warning<=record['capture_wall'] else None
                assert record.get('capacity_warning_wall')==expected
                pi = record['pose_index']
                if pi >= 0:
                    pose = pose_sources[side][pi]
                    assert pose['wall'] == record['pose_receipt_wall'] <= record['capture_wall']+1e-6
                    assert pose['sim'] == record['pose_sim'] <= record['sim']+1e-6
                index = record['map_index']
                if index < 0:
                    assert record['map_stamp'] is None
                    continue
                raw = sources[side][index]
                assert raw['wall'] == record['map_receipt_wall']
                assert raw['sim'] == record['map_receipt_sim']
                assert raw['stamp'] == record['map_stamp']
                assert raw['wall'] <= record['capture_wall']+1e-6
                assert raw['sim'] <= record['sim']+1e-6
                assert raw['stamp'] <= record['sim']+1e-6
                checked += 1
    source_summary = read(args.raw/'rendered/verification.json')
    source_hashes = read(args.raw/'rendered/evidence-sha256.json')
    before, after = read(args.raw/'runtime-before.json'), read(args.raw/'runtime-after.json')
    assert before == after and after['verified']
    assert read(OUTPUT/'reference-checks.json')['passed']
    summaries = read(OUTPUT/'summary.json')
    assert summaries[0]['tested_commit'] == summaries[1]['tested_commit'] == after['identity']['git_commit']
    for name, summary in zip(('baseline', 'candidate'), summaries):
        assert summary['radius'] is None and summary['budget_seconds'] == args.budget
        assert summary['capture']['error'] is None and summary['capture']['encoder_exit'] == 0
        assert summary['process_exits']['video']==0 and summary['process_exits']['backend']==0
        assert summary['process_exits']['driver']==int(summary['outcome'] not in ('passed','budget_completed'))
        # Final statistics and final video map must describe the same snapshot.
        last = json.loads((OUTPUT/'overlay-timing.jsonl').read_text(encoding='utf-8').splitlines()[-1])
        side = 0 if name == 'baseline' else 1
        assert last['sides'][side]['map_stamp'] == summary['stamp']
    for path in (OUTPUT/'v3-exploration-mapping-comparison.mp4',):
        decoded = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(path), '-f', 'null', '-'], capture_output=True, text=True)
        assert decoded.returncode == 0 and not decoded.stderr.strip()
    receipt = dict(passed=True, audited_map_displays=checked, future_data_violations=0,
                   source_videos=source_summary, verified_product_sources=len(read(args.raw/'runtime-source.json')['source_sha256']),
                   verified_install_files=len(after['installed']), frozen_runtime_unchanged=True,
                   final_statistics_match_video_snapshot=True)
    if (OUTPUT/'orphan-cleanup.json').exists():
        receipt['resource_isolation_limitation']=read(OUTPUT/'orphan-cleanup.json')['performance_comparison_limitation']
        receipt['strict_resource_isolation_verified']=False
    (OUTPUT/'verification.json').write_text(json.dumps(receipt, indent=2)+'\n', encoding='utf-8', newline='\n')
    paths += [args.raw/'runtime-source.json', args.raw/'runtime-before.json', args.raw/'runtime-after.json']
    paths += [OUTPUT.parent/'turn_minimap_20261111/render_video.py',
              OUTPUT.parent/'autonomy_terrain_comparison_20261116/verify.py',
              Path(__file__).resolve(),Path(__file__).with_name('compose.py'),Path(__file__).with_name('map_reference.py')]
    paths += [p for p in OUTPUT.iterdir() if p.suffix in ('.json', '.jsonl', '.png', '.mp4', '.py', '.npz', '.md') and p.name != 'evidence-sha256.json']
    for path in paths:
        source_hashes[str(path)] = sha(path)
    (OUTPUT/'evidence-sha256.json').write_text(json.dumps(source_hashes, indent=2)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(receipt), flush=True)


if __name__ == '__main__':
    main()
