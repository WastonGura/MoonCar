"""Run the predeclared paired step checks; failed trials remain in their slots."""

import json
from pathlib import Path
import subprocess


root = Path(__file__).resolve().parent
plan = json.loads((root/"plan.json").read_text())
cases = [c for c in plan["cases"] if c["name"].startswith("repeat32g_")]
assert len(cases) == 10
seeds = {c["seed"] for c in cases}
assert len(seeds) == 5 and all({c["dt_s"] for c in cases if c["seed"] == s} == {4e-6, 8e-6} for s in seeds)
assert all(c["mass_g"] == 32 and c["mpi_ranks"] == 4 and c["diameter_mm"] == 2 for c in cases)
codes = []
for case in cases:
    name = case["name"]
    assert not (root/name).exists(), "Use a new batch for reruns"
    command = ["mpiexec.openmpi", "--allow-run-as-root", "-n", "4", "python", "pile.py",
               "--out", name, "--mass-g", "32", "--seconds", "8", "--seed", str(case["seed"]),
               "--dt", str(case["dt_s"]), "--expect-ranks", "4"]
    with (root/(name+".log")).open("w") as log:
        code = subprocess.run(command, cwd=root, stdout=log, stderr=subprocess.STDOUT).returncode
    (root/(name+".exit")).write_text(str(code)+"\n")
    codes.append(code)
    print(json.dumps({"trial": name, "exit_code": code}), flush=True)
raise SystemExit(0 if all(c == 0 for c in codes) else 1)
