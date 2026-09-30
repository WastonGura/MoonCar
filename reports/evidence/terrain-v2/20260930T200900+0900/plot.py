"""Plot the figure-read paper trend beside this rigid-contact feasibility test."""

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


here = Path(__file__).resolve().parent
with (here / "rigid_response.csv").open(encoding="utf-8") as f:
    rows = list(csv.DictReader(f))
with (here / "ground_boundary.csv").open(encoding="utf-8") as f:
    boundary = list(csv.DictReader(f))

x = [float(row["slip_command_pct"]) for row in rows]
fig, (left, right) = plt.subplots(1, 2, figsize=(10, 4), layout="constrained")
left.plot(x, [float(row["paper_FJS1_fig5_approx"]) for row in rows], "o-", color="#bd493f", label="FJS-1 paper Fig. 5b (approx.)")
left.plot(x, [float(row["traction_coeff_measured"]) for row in rows], "s-", color="#235b88", label="Rigid Bullet, ground μ=0.6")
left.set(xlabel="Commanded slip (%)", ylabel="Mean Fx / mean Fz", title="Slip response", xlim=(-2, 52), ylim=(-0.2, 0.5))
left.legend(fontsize=8)
left.grid(alpha=0.2)
for mu, color in ((0.2, "#75a2af"), (0.6, "#235b88"), (1.2, "#243a54")):
    points = [row for row in boundary if float(row["sdf_mu_mu2"]) == mu]
    right.plot([float(row["slip_pct"]) for row in points],
               [float(row["traction_coeff_measured"]) for row in points],
               "o-", color=color, label=f"ground μ=μ2={mu}")
right.set(xlabel="Commanded slip (%)", ylabel="Mean Fx / mean Fz", title="Ground friction endpoints", xlim=(-2, 52))
right.legend(fontsize=8)
right.grid(alpha=0.2)
fig.savefig(here / "rigid_vs_paper.png", dpi=180)

fig, ax = plt.subplots(figsize=(9, 3.5), layout="constrained")
for case, color in (("slip_00", "#235b88"), ("slip_50", "#bd493f")):
    samples = [json.loads(line) for line in (here / f"{case}_x_force.jsonl").read_text().splitlines()]
    times = [int(row["header"]["stamp"].get("sec", 0)) +
             int(row["header"]["stamp"].get("nsec", 0)) * 1e-9 for row in samples]
    ax.plot(times, [row["force"].get("x", 0) for row in samples], color=color, alpha=0.7, label=case)
ax.set(xlabel="Simulation time (s)", ylabel="Rail force x (N)", title="Force varies within each run")
ax.legend()
ax.grid(alpha=0.2)
fig.savefig(here / "force_time.png", dpi=180)
