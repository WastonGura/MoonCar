"""Place both complete recordings side by side on their shared simulation clock."""
import hashlib
import json
from pathlib import Path
import subprocess


def main():
    root = Path(__file__).resolve().parent
    names = ('baseline', 'candidate')
    metadata = [json.loads((root/name/'summary.json').read_text(encoding='utf-8')) for name in names]
    videos = [root/'baseline/baseline-autonomy.mp4', root/'candidate/candidate-autonomy.mp4']
    fps = metadata[0]['fps']
    assert all(row['fps'] == fps for row in metadata)
    origin = min(row['sim_range'][0] for row in metadata)
    leading = [round((row['sim_range'][0]-origin)*fps) for row in metadata]
    frames = max(row['frames']+pad for row, pad in zip(metadata, leading))
    trailing = [frames-row['frames']-pad for row, pad in zip(metadata, leading)]
    filters = [f'[{i}:v]setpts=PTS-STARTPTS,tpad=start_mode=clone:start={start}:stop_mode=clone:stop={end}[v{i}]'
               for i, (start, end) in enumerate(zip(leading, trailing))]
    filters.append('[v0][v1]hstack=inputs=2:shortest=1[out]')
    output = root/'v2-autonomy-side-by-side.mp4'
    subprocess.run(['ffmpeg', '-hide_banner', '-loglevel', 'error', '-y',
                    '-i', str(videos[0]), '-i', str(videos[1]),
                    '-filter_complex', ';'.join(filters), '-map', '[out]', '-an',
                    '-c:v', 'libx264', '-threads', '4', '-preset', 'fast', '-crf', '20',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)], check=True)
    decoded = subprocess.run(['ffmpeg', '-v', 'error', '-i', str(output), '-f', 'null', '-'],
                             capture_output=True, text=True, check=True)
    if decoded.stderr.strip():
        raise RuntimeError(decoded.stderr)
    probe = json.loads(subprocess.check_output([
        'ffprobe', '-v', 'error', '-select_streams', 'v:0', '-show_entries',
        'stream=width,height,nb_frames,r_frame_rate:format=duration', '-of', 'json', str(output)]))
    assert int(probe['streams'][0]['nb_frames']) == frames
    subprocess.run(['ffmpeg', '-v', 'error', '-y', '-ss', '90', '-i', str(output),
                    '-frames:v', '1', '-update', '1', str(root/'side-by-side-preview.png')], check=True)
    digest = hashlib.sha256()
    with output.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            digest.update(chunk)
    summary = dict(video=output.name, left='baseline', right='elevation_neighborhood_v1',
                   fps=fps, frames=frames, seconds=frames/fps, common_sim_start=origin,
                   leading_hold_frames=leading, trailing_hold_frames=trailing,
                   alignment_rounding_error_seconds=[pad/fps-(row['sim_range'][0]-origin)
                                                     for row, pad in zip(metadata, leading)],
                   policy='Complete original videos retained; shared simulation clock, endpoint holds only.',
                   complete_decode_exit=decoded.returncode, probe=probe, sha256=digest.hexdigest())
    (root/'combined-summary.json').write_text(json.dumps(summary, indent=2)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
