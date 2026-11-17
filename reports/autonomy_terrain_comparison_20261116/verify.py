"""Decode complete videos and audit their causal map/pose overlays."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess


def rows(path):
    result=[]
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            if not line.strip():continue
            row=json.loads(line)
            if path.name=='states.jsonl':
                row=dict(wall=row['wall'],state=dict(motion=dict(pose=row['state']['motion'].get('pose'))))
            elif path.name=='navigation-grids.jsonl':
                row=dict(wall=row['wall'],grid=dict(stamp=row['grid'].get('stamp',0)))
            result.append(row)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--raw', type=Path, required=True)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    checks = []
    hashes = {}
    for name in ('baseline', 'candidate'):
        raw, report = args.raw/name, args.report/name
        states = rows(raw/'case/states.jsonl')
        poses = [r for r in states if r['state']['motion'].get('pose')]
        grids = rows(raw/'case/navigation-grids.jsonl')
        alignment = rows(report/'frame-alignment.jsonl')
        violations = []
        for index, frame in enumerate(alignment):
            wall, stamp = frame['capture_wall'], frame['sim']
            for key, source in (('state_index', states), ('grid_index', grids), ('pose_index', poses)):
                selected = frame[key]
                if selected >= 0 and source[selected]['wall'] > wall+1e-6:
                    violations.append([index, key, 'future receipt'])
            selected = frame['grid_index']
            if selected >= 0 and grids[selected]['grid'].get('stamp', 0) > stamp+1e-6:
                violations.append([index, 'grid_index', 'future simulation stamp'])
        summary = json.loads((report/'summary.json').read_text(encoding='utf-8'))
        video = Path(summary['video'])
        decoded = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(video), '-f', 'null', '-'],
                                 capture_output=True, text=True)
        if decoded.returncode or decoded.stderr.strip() or violations:
            raise RuntimeError(f'{name}: video/overlay audit failed: {decoded.stderr} {violations[:5]}')
        probe = json.loads(subprocess.check_output([
            'ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
            'stream=width,height,nb_frames,r_frame_rate:format=duration', '-of', 'json', str(video)]))
        checks.append(dict(strategy=name, audited_frames=len(alignment), future_data_violations=0,
                           complete_decode_exit=decoded.returncode, video=probe))
        selected_files = [raw/'case/result.json', raw/'video/recording.json',
                          raw/'video/frames.jsonl', raw/'video/scene-walltime.mp4',
                          raw/'case/states.jsonl', raw/'case/navigation-grids.jsonl',
                          raw/'case/camera/ready.json', raw/'case/observer/truth.jsonl',
                          raw/'case/observer/alignment.jsonl', raw/'case/observer/samples.jsonl',
                          video, report/'summary.json', report/'frame-alignment.jsonl']
        for path in selected_files:
            digest = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(chunk)
            hashes[str(path)] = digest.hexdigest()
    (args.report/'verification.json').write_text(json.dumps(checks, indent=2)+'\n', encoding='utf-8', newline='\n')
    (args.report/'evidence-sha256.json').write_text(json.dumps(hashes, indent=2)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(checks), flush=True)


if __name__ == '__main__':
    main()
