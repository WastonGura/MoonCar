"""Replay the entire retained prototype run, with explicit footage provenance."""

import bisect
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

from add_wheel_detail import EVIDENCE, HEIGHT, WIDTH, interpolate, load_wheels, panel, read_rows


ROOT = EVIDENCE / "feedback-turn-video-20261105/trial-run"
OUTPUT = Path(__file__).resolve().parent
FPS = 20
BG, CARD, GRID = "#111925", "#182434", "#31455b"
WHITE, MUTED, CYAN, YELLOW, RED = "#edf3fc", "#acbfd4", "#7cdded", "#ffd17e", "#ff8b84"
ZOOM = (904, 89, 1260, 452)


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def draw_vehicle(draw, project, pose, size):
    x, y = pose["xyz"][:2]
    c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])

    def point(dx, dy):
        return project(x + size * (dx * c - dy * s), y + size * (dx * s + dy * c))

    def rectangle(dx, dy, hx, hy, color):
        draw.polygon(
            [point(dx+sx*hx, dy+sy*hy) for sx, sy in ((-1,-1),(1,-1),(1,1),(-1,1))],
            fill=color, outline=WHITE,
        )

    rectangle(0, 0, .16, .125, "#899fb7")
    for dx in (-.157, .157):
        for dy in (-.16555, .16555):
            rectangle(dx, dy, .076, .025, "#263547")
    draw.line((point(0, 0), point(.29, 0)), fill=YELLOW, width=4)


def render():
    observer = ROOT / "case/observer"
    truth = list(read_rows(observer / "truth.jsonl"))
    stamps = np.asarray([row["stamp"] for row in truth])
    poses = np.asarray([row["xyz"] for row in truth])
    autonomy = read_json(ROOT / "case/result.json")["autonomy"]
    actions = autonomy["history"]
    starts = [autonomy["started"] + row["started_seconds"] for row in actions]
    ends = [row["end_sim"] for row in actions]
    turn_start, stop = starts[-1], ends[-1]
    wheels, turn_anchor = load_wheels(observer, turn_start)
    first = max(stamps[0], *(data[0][0] for data in wheels.values()))
    body_end = stamps[-1]
    end = min(data[0][-1] for data in wheels.values())
    tag_end = min(data[0][np.isfinite(data[3]).all(axis=1)][-1] for data in wheels.values())
    slow_start = ends[2]
    for wheel_time, angle, height, _, _ in wheels.values():
        angle -= interpolate(wheel_time, angle, first)[0]
        height -= interpolate(wheel_time, height, first)[0]
    turn_yaw = truth[np.argmin(np.abs(stamps - turn_start))]["yaw"]
    intervals = (
        (first, starts[0], "启动观察与建图", "#6b8ca7"),
        (starts[0], starts[1], "第 1 段", "#477995"),
        (starts[1], starts[2], "第 2 段", "#498ea2"),
        (starts[2], slow_start, "第 3 段", "#5ba3ad"),
        (slow_start, turn_start, "准备转向", "#b68d51"),
        (turn_start, stop, "转向", "#bc655e"),
    )
    commands = [row for row in read_rows(observer / "samples.jsonl") if
                row.get("topic") == "/mooncar/cmd_vel_safe" and row.get("sim") is not None]
    command_times = [row["sim"] for row in commands]
    state_pairs = [(row["wall"], row["state"]["motion"].get("diagnostics", {}).get("sim_stamp"))
                   for row in read_rows(ROOT / "case/states.jsonl")]
    wall, sim = zip(*[(wall, sim) for wall, sim in state_pairs if sim is not None])
    capture_dir = ROOT / "actual-video-run"
    recorded = list(read_rows(capture_dir / "frames.jsonl"))
    recording = read_json(capture_dir / "recording.json")
    raw_width, raw_height = recording["width"], recording["height"]
    aligned = [row for row in recorded if wall[0] <= row["capture_wall"] <= wall[-1]]
    video_times = np.interp([row["capture_wall"] for row in aligned], wall, sim)
    if not len(video_times) or np.any(np.diff(video_times) < 0):
        raise ValueError("The recording does not have an ordered simulation timeline")
    raw_count = aligned[-1]["frame"] + 1
    fonts = {size: ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", size)
             for size in (15, 17, 20, 22, 24, 28, 32)}

    def text(draw, xy, value, size=20, color=WHITE):
        draw.text(xy, value, font=fonts[size], fill=color)

    def scene(time, picture, held):
        index = max(0, bisect.bisect_right(stamps, time) - 1)
        pose = truth[index]
        x, y, z = pose["xyz"]
        view_center = np.asarray([x, y]) if time < slow_start else turn_anchor
        yaw = math.degrees(math.remainder(pose["yaw"] - turn_yaw, 2 * math.pi))
        cmd = commands[max(0, bisect.bisect_right(command_times, time) - 1)]
        active = bisect.bisect_right(starts, time) - 1
        if time >= stop:
            heading_label = "车体末帧转过" if time > body_end else "本次累计转过"
            stage, status, accent = "触发停车", f"{heading_label}约 {yaw:.1f}°", RED
        elif active == 3:
            stage, status, accent = "尝试左转 90°", f"已转 {yaw:.1f}°", YELLOW
        elif active < 0:
            stage, status, accent = "启动观察与建图", "车体停在起点", CYAN
        elif time < ends[active]:
            target = actions[active]["action"][1]
            stage, status, accent = f"第 {active+1} 段前进", f"目标位置  X = {target:.1f} m", CYAN
        else:
            stage, status, accent = f"第 {active+1} 段完成", "等待下一动作", CYAN
        im = Image.new("RGB", (WIDTH, 800), BG)
        d = ImageDraw.Draw(im)
        text(d, (20, 9), "新原型全程   V2 坡道", 32)
        speed = "1× 仿真时间" if time < slow_start else "4 倍慢放"
        text(d, (20, 54), f"仿真 {time:06.2f} s  /  {end:.2f} s    {speed}", 20, MUTED)
        if held:
            text(d, (625, 54), "记录结束   停帧", 20, YELLOW)
        d.rounded_rectangle((20, 89, 884, 649), radius=10, fill=CARD)
        if picture is None:
            text(d, (35, 99), "车体轨迹回放   仿真位置与朝向记录", 22)
            text(d, (35, 135), "画面实录从仿真 28.79 s 开始", 17, MUTED)
            def wide(xpos, ypos):
                return (82 + xpos * 170, 342 - ypos * 170)
            for coord in np.arange(0, 4.6, .5):
                px = wide(coord, 0)[0]
                d.line((px, 180, px, 491), fill=GRID)
                text(d, (px - 10, 496), f"{coord:g}", 15, MUTED)
            for coord in (-.75, -.5, -.25, 0., .25, .5, .75):
                py = wide(0, coord)[1]
                d.line((56, py, 862, py), fill=GRID)
            trace = [wide(*point[:2]) for point in poses[:index+1:12]]
            if len(trace) > 1:
                d.line(trace, fill=CYAN, width=4)
            for target in (1., 2.5, 4.):
                px, py = wide(target, 0)
                d.ellipse((px-6, py-6, px+6, py+6), outline=YELLOW, width=2)
                text(d, (px-24, py+28), f"{target:g} m", 15, YELLOW)
            draw_vehicle(d, wide, pose, 1.)
        else:
            capture_label = "Gazebo 实录   按仿真时间对齐" if time <= video_times[-1] else "Gazebo 实录末帧   右侧数据继续至记录结束"
            text(d, (35, 99), capture_label, 22)
            crop = picture.crop((250, 185, 850, 615))
            im.paste(ImageOps.fit(crop, (864, 400), method=Image.Resampling.LANCZOS), (20, 141))
            d = ImageDraw.Draw(im)

        text(d, (35, 550), "车体高度沿路线变化", 17, MUTED)
        d.line((40, 631, 862, 631), fill=GRID)
        def profile(p):
            return (45 + p[0] * 190, 630 - p[2] * 170)
        d.line([profile(p) for p in poses[::12]], fill="#65798e", width=2)
        profile_trace = [profile(p) for p in poses[:index+1:12]]
        if len(profile_trace) > 1:
            d.line(profile_trace, fill=CYAN, width=3)
        px, py = profile(pose["xyz"])
        d.ellipse((px-5, py-5, px+5, py+5), fill=YELLOW)

        d.rounded_rectangle(ZOOM, radius=10, fill=CARD)
        def zoom(xpos, ypos):
            return 1082 + (xpos-view_center[0]) * (356/1.8), 270.5 - (ypos-view_center[1]) * (356/1.8)
        for coord in np.arange(-.8, .81, .2):
            px = zoom(view_center[0]+coord, view_center[1])[0]
            py = zoom(view_center[0], view_center[1]+coord)[1]
            d.line((px, 128, px, 437), fill=GRID)
            d.line((920, py, 1243, py), fill=GRID)
        closeup = "四轮俯视   车体末帧定格" if time > body_end else "四轮俯视   按姿态记录绘制"
        text(d, (916, 99), closeup, 17)
        draw_vehicle(d, zoom, pose, 1.)
        text(d, (908, 465), stage, 28, accent)
        text(d, (908, 506), status, 20)
        text(d, (908, 547), f"位置 X {x:.2f} m   高度 {z:.2f} m", 17, MUTED)
        text(d, (908, 578), f"线速度指令  {cmd['linear']:+.3f} m/s", 17, MUTED)
        text(d, (908, 606), f"角速度指令  {cmd['angular']:+.3f} rad/s", 17, MUTED)
        if time >= stop:
            text(d, (908, 634), "预测扫掠触及地图障碍", 17, RED)
        if time > body_end:
            text(d, (908, 661), "车轮继续减速   轮心横移缺测", 17, YELLOW)
        text(d, (20, 667), "全程时间轴", 20)
        text(d, (175, 668), "启动观察  →  三段前进  →  转向  →  触发停车与减速", 17, MUTED)
        def timeline(t):
            return 24 + (t-first) / (end-first) * 1230
        for begin, finish, label, color in intervals:
            a, b = timeline(begin), timeline(finish)
            d.rectangle((a, 706, b, 727), fill=color)
            if b-a > 90:
                text(d, (a+5, 735), label, 15, MUTED)
        capture_x = timeline(video_times[0])
        d.line((capture_x, 700, capture_x, 746), fill=CYAN, width=2)
        text(d, (capture_x - 60, 761), "实录开始", 15, CYAN)
        px = timeline(time)
        d.line((px, 696, px, 733), fill=WHITE, width=3)
        return panel(im, wheels, view_center, time, fonts, True, reference="全程起点",
                     position_time=min(time, tag_end))

    timeline = list(np.arange(first, slow_start, 1/FPS)) + list(np.arange(slow_start, end, .25/FPS))
    timeline += [end] * (2 * FPS)
    decoder = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-i", str(capture_dir / "scene-walltime.mp4"), "-frames:v", str(raw_count),
         "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
        stdout=subprocess.PIPE,
    )
    target = OUTPUT / "feedback-turn-full-process.mp4"
    encoder = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}",
         "-r", str(FPS), "-i", "pipe:0", "-c:v", "libx264", "-threads", "2", "-preset", "fast",
         "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(target)],
        stdin=subprocess.PIPE,
    )
    previous = -1
    picture = None
    previews = [(starts[1]+1, "full-approach-preview.png"), (turn_start+.85, "full-turn-preview.png"),
                (end, "full-stop-preview.png")]
    try:
        for frame, time in enumerate(timeline):
            if time >= video_times[0]:
                aligned_index = int(np.argmin(np.abs(video_times - time)))
                wanted = aligned[aligned_index]["frame"]
                while previous < wanted:
                    blob = decoder.stdout.read(raw_width*raw_height*3)
                    if len(blob) != raw_width*raw_height*3:
                        raise ValueError("Screen recording ended before the aligned frame")
                    picture = Image.frombytes("RGB", (raw_width, raw_height), blob)
                    previous += 1
            still = scene(time, picture, frame >= len(timeline)-2*FPS)
            if previews and time >= previews[0][0]:
                still.save(OUTPUT / previews.pop(0)[1])
            encoder.stdin.write(still.tobytes())
        decoder.stdout.read()
    finally:
        decoder.stdout.close()
        encoder.stdin.close()
        decoder_code = decoder.wait(timeout=30)
        encoder_code = encoder.wait(timeout=30)
    if decoder_code or encoder_code:
        raise RuntimeError("Full video encoding or source decode failed")
    print(json.dumps({"video": str(target), "frames": len(timeline), "seconds": len(timeline)/FPS,
                      "sim_range": [float(first), float(end)],
                      "first_recorded_frame_sim": float(video_times[0]),
                      "turn_start_sim": turn_start, "stop_sim": stop}), flush=True)


if __name__ == "__main__":
    render()
