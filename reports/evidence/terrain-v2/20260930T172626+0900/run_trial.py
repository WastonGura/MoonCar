"""Run one frozen, headless wheel-ground sensitivity trial."""

import json
from pathlib import Path
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
WORLD = "mooncar_ramp_v2"
CONTROL = f"/world/{WORLD}/control"
WHEELS = {f"{axle}_{side}_wheel_joint" for axle in ("front", "rear") for side in ("left", "right")}


def command(*args, timeout=20):
    result = subprocess.run(args, text=True, capture_output=True, timeout=timeout, check=True)
    return result.stdout.strip()


def topic(name):
    return json.loads(command("gz", "topic", "-e", "--json-output", "-n", "1", "-t", name))


def control(request):
    result = command("gz", "service", "-s", CONTROL, "--reqtype", "gz.msgs.WorldControl",
                     "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", request)
    if "data: true" not in result:
        raise RuntimeError(f"Gazebo rejected {request!r}: {result}")


def wait_iterations(target):
    deadline = time.monotonic() + 45
    while time.monotonic() < deadline:
        stats = topic(f"/world/{WORLD}/stats")
        iterations = int(stats["iterations"])
        if iterations >= target and stats.get("paused"):
            if iterations != target:
                raise RuntimeError(f"Expected iteration {target}, got {iterations}")
            return
        time.sleep(0.2)
    raise TimeoutError(f"World did not reach iteration {target}")


def sample():
    stats = topic(f"/world/{WORLD}/stats")
    poses = topic(f"/world/{WORLD}/pose/info")["pose"]
    pose = next(item for item in poses if item["name"] == "mooncar_r5")["position"]
    joints = topic("/joint_states")["joint"]
    wheels = {item["name"]: item["axis1"]["position"] for item in joints if item["name"] in WHEELS}
    assert set(wheels) == WHEELS
    return {"iterations": int(stats["iterations"]), "sim_seconds": int(stats["iterations"]) * 0.002,
            "x": pose.get("x", 0.0), "y": pose.get("y", 0.0), "z": pose.get("z", 0.0),
            "wheel_rad": wheels}


def main(case):
    if case not in ("baseline", "low_friction"):
        raise ValueError(case)
    with (HERE / f"{case}_server.log").open("w", encoding="utf-8") as log:
        server = subprocess.Popen(["gz", "sim", "-s", "-v", "3", str(HERE / f"{case}.sdf")],
                                  stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 45
            while CONTROL not in command("gz", "service", "-l"):
                if server.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("Gazebo control service unavailable")
                time.sleep(0.5)
            control("pause: true")
            control("pause: true, reset: {all: true}")
            control("pause: true, multi_step: 1000")
            wait_iterations(1000)
            with (HERE / f"{case}_samples.jsonl").open("w", encoding="utf-8") as output:
                for block in range(11):
                    if block:
                        command("gz", "topic", "-t", "/cmd_vel", "-m", "gz.msgs.Twist",
                                "-p", "linear: {x: 0.25}")
                        control("pause: true, multi_step: 1000")
                        wait_iterations(1000 + block * 1000)
                    row = sample()
                    assert row["iterations"] == 1000 + block * 1000
                    print(json.dumps(row, ensure_ascii=False), file=output, flush=True)
                    print(f"{case} block {block}/10 x={row['x']:.3f} m", flush=True)
        finally:
            server.terminate()
            try:
                server.wait(timeout=10)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait()


if __name__ == "__main__":
    main(sys.argv[1])
