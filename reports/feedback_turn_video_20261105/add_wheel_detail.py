"""Add four-wheel telemetry to the two retained turn videos using their frame clocks."""

import argparse
import json
import math
from pathlib import Path
import subprocess

import numpy as np
from PIL import Image, ImageDraw, ImageFont


REPO = Path(__file__).resolve().parents[2]
EVIDENCE = REPO / "tmp/ros_acceptance"
REPORT = REPO / "reports/feedback_turn_video_20261105"
WHEELS = (
    ("front_left", "左前轮", "FL", "#ffd17e"),
    ("front_right", "右前轮", "FR", "#7cdded"),
    ("rear_left", "左后轮", "RL", "#ffd17e"),
    ("rear_right", "右后轮", "RR", "#7cdded"),
)
WIDTH, ORIGINAL_HEIGHT, HEIGHT = 1280, 800, 1072
AVERAGING_SECONDS = 0.10
MAX_INTERPOLATION_GAP = 0.06


def read_rows(path):
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def interpolate(stamps, values, query):
    """Interpolate local samples, leaving missing intervals and extrapolation empty."""
    query = np.asarray(query)
    columns = np.asarray(values).reshape(len(stamps), -1)
    result = np.stack([np.interp(query, stamps, col) for col in columns.T], axis=-1)
    right = np.searchsorted(stamps, query, side="left").clip(1, len(stamps) - 1)
    left = right - 1
    gap = stamps[right] - stamps[left]
    exact = (query == stamps[left]) | (query == stamps[right])
    valid = (query >= stamps[0]) & (query <= stamps[-1]) & (
        exact | (gap <= MAX_INTERPOLATION_GAP)
    )
    return np.where(valid[..., None], result, np.nan)


def rotate(quaternion, vector):
    xyz = quaternion[:, :3]
    twice_cross = 2 * np.cross(xyz, vector)
    return vector + quaternion[:, 3:] * twice_cross + np.cross(xyz, twice_cross)


def load_wheels(observer, turn_start):
    joint_rows = {name: {} for name, *_ in WHEELS}
    carrier_rows = {name: {} for name, *_ in WHEELS}
    base_offset = None
    for row in read_rows(observer / "tf.jsonl"):
        if (row["parent"], row["child"]) == ("base_footprint", "base_link"):
            if not np.allclose(row["q"], [0, 0, 0, 1]):
                raise ValueError("Expected a translation-only base offset")
            base_offset = np.asarray(row["xyz"])
        for name, *_ in WHEELS:
            carrier = name + "_wheel_carrier_link"
            wheel = name + "_wheel_link"
            if (row["parent"], row["child"]) == (carrier, wheel):
                if not np.allclose(row["xyz"], [0, 0, 0]):
                    raise ValueError("Wheel center and carrier origin differ")
                joint_rows[name][row["stamp_ns"]] = row
            elif (row["parent"], row["child"]) == ("base_link", carrier):
                carrier_rows[name][row["stamp_ns"]] = row
    if base_offset is None:
        raise ValueError("Base offset missing from TF")

    truth = {row["stamp_ns"]: row for row in read_rows(observer / "truth.jsonl")}
    ordered = [truth[key] for key in sorted(truth)]
    truth_time = np.asarray([row["stamp"] for row in ordered])
    truth_xyz = np.asarray([row["xyz"] for row in ordered])
    truth_q = np.asarray([row["q"] for row in ordered])
    sign = np.where(np.sum(truth_q[:-1] * truth_q[1:], axis=1) < 0, -1, 1)
    truth_q *= np.r_[1, np.cumprod(sign)][:, None]
    wheels = {}
    for name, *_ in WHEELS:
        keys = sorted(joint_rows[name].keys() & carrier_rows[name].keys())
        if len(keys) < 2:
            raise ValueError(f"Missing paired wheel/carrier samples for {name}")
        stamps = np.asarray(keys) * 1e-9
        joint_q = np.asarray([joint_rows[name][key]["q"] for key in keys])
        if not np.allclose(joint_q[:, [0, 2]], 0, atol=1e-6):
            raise ValueError(f"Unexpected rotation axis for {name}")
        angle = np.unwrap(2 * np.arctan2(joint_q[:, 1], joint_q[:, 3]))
        local = np.asarray([carrier_rows[name][key]["xyz"] for key in keys])
        quat = interpolate(truth_time, truth_q, stamps)
        quat /= np.linalg.norm(quat, axis=1, keepdims=True)
        center = interpolate(truth_time, truth_xyz, stamps) + rotate(quat, local + base_offset)
        axle = rotate(quat, np.array([0.0, 1.0, 0.0]))
        start_angle = interpolate(stamps, angle, turn_start)[0]
        start_height = interpolate(stamps, local[:, 2], turn_start)[0]
        if not all(np.isfinite([start_angle, start_height])):
            raise ValueError(f"No wheel reference at the start of turn for {name}")
        wheels[name] = (stamps, angle - start_angle, local[:, 2] - start_height, center, axle)
    anchor = truth_xyz[np.argmin(np.abs(truth_time - turn_start)), :2]
    return wheels, anchor


def sample(wheel, time):
    stamps, angle, height, center, axle = wheel
    if time < stamps[0] or time > stamps[-1]:
        return (math.nan,) * 4
    # Shift the whole averaging interval at a capture edge, rather than extrapolating.
    left = np.clip(time - AVERAGING_SECONDS / 2, stamps[0], stamps[-1] - AVERAGING_SECONDS)
    right = left + AVERAGING_SECONDS
    angle_endpoints = interpolate(stamps, angle, [left, right])[:, 0]
    hub_endpoints = interpolate(stamps, center, [left, right])
    axis = interpolate(stamps, axle, time)
    rpm = (angle_endpoints[1] - angle_endpoints[0]) * 60 / (2 * math.pi * AVERAGING_SECONDS)
    lateral = (hub_endpoints[1] - hub_endpoints[0]) @ axis / AVERAGING_SECONDS
    degrees = math.degrees(interpolate(stamps, angle, time)[0])
    millimeters = interpolate(stamps, height, time)[0] * 1000
    return rpm, lateral * 100, degrees, millimeters


def panel(frame, wheels, anchor, time, fonts, candidate, reference="转向起点", position_time=None):
    result = Image.new("RGB", (WIDTH, HEIGHT), "#111925")
    result.paste(frame, (0, 0))
    draw = ImageDraw.Draw(result)
    white, subdued = "#edf3fc", "#b5c5d8"
    draw.line((20, 802, 1260, 802), fill="#40546b", width=2)
    draw.text((20, 813), "四轮运动   关节记录回放", font=fonts[24], fill=white)
    draw.text(
        (20, 846),
        f"轮速正值为前滚，负值为反滚。横移按车体左为正。累计滚动与悬架位移相对{reference}。",
        font=fonts[17],
        fill=subdued,
    )
    if candidate:
        box = (904, 89, 1260, 452)
    else:
        box = (20, 89, 884, 649)
        draw.rectangle((902, 162, 1265, 192), fill="#111925")
        draw.text((908, 167), "上方轮位示意，下方为关节记录", font=fonts[15], fill=white)
    x0, y0, x1, y1 = box
    scale = min(x1 - x0, y1 - y0) / 1.8
    for index, (name, label, tag, color) in enumerate(WHEELS):
        stamps, _, _, centers, _ = wheels[name]
        hub_xy = interpolate(stamps, centers, time if position_time is None else position_time)[:2]
        if np.isfinite(hub_xy).all():
            px = (x0 + x1) / 2 + (hub_xy[0] - anchor[0]) * scale
            py = (y0 + y1) / 2 - (hub_xy[1] - anchor[1]) * scale
            if x0 + 16 < px < x1 - 16 and y0 + 11 < py < y1 - 11:
                draw.rounded_rectangle((px-14, py-10, px+14, py+10), radius=4, fill=color)
                draw.text((px, py), tag, font=fonts[15], fill="#111925", anchor="mm")
        x = 20 + index * 314
        draw.rounded_rectangle((x, 877, x + 298, 1053), radius=12, fill="#1c2b3e")
        draw.text((x + 14, 886), f"{label} {tag}", font=fonts[22], fill=color)
        rpm, lateral, angle, height = sample(wheels[name], time)
        cx, cy, radius = x + 52, 984, 30
        draw.ellipse((cx-radius, cy-radius, cx+radius, cy+radius), fill="#111925", outline="#75869b", width=2)
        if math.isfinite(angle):
            theta = math.radians(angle)
            px, py = cx + radius * math.sin(theta), cy - radius * math.cos(theta)
            draw.line((cx, cy, px, py), fill=color, width=3)
            draw.ellipse((px-5, py-5, px+5, py+5), fill=color)
        def number(value, precision):
            return f"{value:+.{precision}f}" if math.isfinite(value) else "—"
        motion = "近停" if abs(rpm) < 0.1 else "前滚" if rpm > 0 else "反滚"
        speed = f"{motion}  {number(rpm, 2)} rpm" if math.isfinite(rpm) else "轮速缺测"
        draw.text((x + 97, 924), speed, font=fonts[20], fill=color)
        for y, line in (
            (955, f"累计滚动  {number(angle, 1)}°"),
            (985, f"横移  {number(lateral, 1)} cm/s"),
            (1015, f"悬架  {number(height, 2)} mm"),
        ):
            draw.text((x + 97, y), line, font=fonts[17], fill=white)
    return result


def render(mode, output, fonts):
    candidate = mode == "candidate"
    trial = (
        EVIDENCE / "feedback-turn-video-20261105/trial-run"
        if candidate
        else EVIDENCE / "camera-mode-comparison-20261104/fixed"
    )
    name = "feedback-turn-diagnostic" if candidate else "baseline-turn-replay"
    metadata = json.loads((REPORT / f"{mode}-summary.json").read_text(encoding="utf-8"))
    wheels, anchor = load_wheels(trial / "case/observer", metadata["turn_start_sim"])
    fps = metadata["fps"]
    hold_frames = fps * 2  # Explicit end hold in the source renderer.
    preview_time = metadata["turn_start_sim"] + (0.85 if candidate else 2.5)
    preview_index = round((preview_time - metadata["start_sim"]) * fps / metadata["simulation_playback_speed"])
    dest = output / f"{name}-four-wheels.mp4"
    source = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-i", str(REPORT / f"{name}.mp4"), "-f", "rawvideo", "-pix_fmt", "rgb24", "pipe:1"],
        stdout=subprocess.PIPE,
    )
    encoder = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24",
         "-s", f"{WIDTH}x{HEIGHT}", "-r", str(fps), "-i", "pipe:0", "-c:v", "libx264",
         "-threads", "2", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
         "-movflags", "+faststart", str(dest)],
        stdin=subprocess.PIPE,
    )
    byte_count = WIDTH * ORIGINAL_HEIGHT * 3
    try:
        for index in range(metadata["frames"]):
            pixels = source.stdout.read(byte_count)
            if len(pixels) != byte_count:
                raise ValueError("The source video does not match its frame manifest")
            if index >= metadata["frames"] - hold_frames:
                time = metadata["end_sim"]
            else:
                time = metadata["start_sim"] + index * metadata["simulation_playback_speed"] / fps
            frame = Image.frombytes("RGB", (WIDTH, ORIGINAL_HEIGHT), pixels)
            composed = panel(frame, wheels, anchor, time, fonts, candidate)
            if index == preview_index:
                composed.save(output / f"{mode}-wheel-preview.png")
                print(json.dumps({"mode": mode, "sim_time": time, "rpm_lateral_angle_suspension": {
                    key: [round(float(v), 3) for v in sample(value, time)] for key, value in wheels.items()
                }}), flush=True)
            encoder.stdin.write(composed.tobytes())
        if source.stdout.read(1):
            raise ValueError("Source video has excess frames")
    finally:
        source.stdout.close()
        encoder.stdin.close()
        decoder_code = source.wait(timeout=30)
        encoder_code = encoder.wait(timeout=30)
        if decoder_code or encoder_code:
            raise RuntimeError("Video decode or encode failed")
    print(dest, flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("baseline", "candidate"))
    args = parser.parse_args()
    output = Path(__file__).resolve().parent
    fonts = {size: ImageFont.truetype("C:/Windows/Fonts/msyh.ttc", size) for size in (15, 17, 20, 22, 24)}
    render(args.mode, output, fonts)


if __name__ == "__main__":
    main()
