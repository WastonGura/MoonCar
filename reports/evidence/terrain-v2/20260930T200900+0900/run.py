"""Capture raw joint forces while stepping one paused Gazebo wheel-bench world."""

import json
import os
from pathlib import Path
import subprocess
import sys
import time


HERE = Path(__file__).resolve().parent
WORLD = "wheel_bench"
CONTROL = f"/world/{WORLD}/control"
TOPICS = {
    "ground_force": f"/world/{WORLD}/model/ground/joint/ground_fixed/sensor/ground_force/forcetorque",
    "x_force": f"/world/{WORLD}/model/rig/joint/translate_x/sensor/x_force/forcetorque",
    "z_force": f"/world/{WORLD}/model/rig/joint/float_z/sensor/z_force/forcetorque",
    "contact": f"/world/{WORLD}/model/ground/link/surface/sensor/ground_contact/contact",
}


def command(*args):
    return subprocess.run(args, capture_output=True, text=True, check=True, timeout=20).stdout.strip()


def control(request):
    result = command("gz", "service", "-s", CONTROL, "--reqtype", "gz.msgs.WorldControl",
                     "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", request)
    if "data: true" not in result:
        raise RuntimeError(result)


def stats():
    return json.loads(command("gz", "topic", "-e", "--json-output", "-n", "1", "-t", f"/world/{WORLD}/stats"))


def run(case):
    assert case in tuple(f"slip_{index * 10:02}" for index in range(6)) + (
        "slip_50_noground", "slip_50_known_force", "slip_50_long",
        "ground_mu02_slip_00", "ground_mu02_slip_50", "ground_mu12_slip_00", "ground_mu12_slip_50")
    os.environ["GZ_PARTITION"] = f"mooncar_wheelbench_{case}"
    if (HERE / f"{case}_server.log").exists():
        raise FileExistsError(f"{case} already has evidence; rerun in a new batch directory")
    steps = 10000 if case in ("slip_50_noground", "slip_50_known_force") else 20000
    if case == "slip_50_long":
        steps = 64000
    files = []
    with (HERE / f"{case}_server.log").open("w", encoding="utf-8") as log:
        server = subprocess.Popen(["gz", "sim", "-s", "-v", "3", str(HERE / f"{case}.sdf")],
                                  stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 30
            while CONTROL not in command("gz", "service", "-l"):
                if server.poll() is not None or time.monotonic() > deadline:
                    raise RuntimeError("Gazebo control service unavailable")
                time.sleep(0.2)
            control("pause: true, reset: {all: true}")
            for label, topic in TOPICS.items():
                output = (HERE / f"{case}_{label}.jsonl").open("w", encoding="utf-8")
                listener = subprocess.Popen(["gz", "topic", "-e", "--json-output", "-t", topic],
                                            stdout=output, stderr=subprocess.PIPE)
                files.append((listener, output))
            time.sleep(1)
            control(f"pause: true, multi_step: {steps}")
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                state = stats()
                if int(state.get("iterations", 0)) == steps and state.get("paused"):
                    break
                time.sleep(0.3)
            else:
                raise RuntimeError(f"Stepping timed out: {state}")
            (HERE / f"{case}_joint_state.json").write_text(
                command("gz", "topic", "-e", "--json-output", "-n", "1", "-t", f"/world/{WORLD}/model/rig/joint_state"),
                encoding="utf-8",
            )
            print(json.dumps(state), flush=True)
        finally:
            for listener, output in files:
                listener.terminate()
                try:
                    listener.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    listener.kill()
                    listener.wait()
                output.close()
            server.terminate()
            try:
                server.wait(timeout=5)
            except subprocess.TimeoutExpired:
                server.kill()
                server.wait()


if __name__ == "__main__":
    run(sys.argv[1])
