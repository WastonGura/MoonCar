"""Plot the figure-read paper trend beside this rigid-contact feasibility test."""

import csv
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
left.plot(x, [float(row["traction_coeff_nominal"]) for row in rows], "s-", color="#235b88", label="Rigid Bullet, ground μ=0.6")
left.set(xlabel="Commanded slip (%)", ylabel="Fx / 24.5 N", title="Slip response", xlim=(-2, 52), ylim=(0, 0.5))
left.legend(fontsize=8)
left.grid(alpha=0.2)
for mu, color in ((0.2, "#75a2af"), (0.6, "#235b88"), (1.2, "#243a54")):
    points = [row for row in boundary if float(row["sdf_mu_mu2"]) == mu]
    right.plot([float(row["slip_pct"]) for row in points],
               [float(row["traction_coeff_nominal"]) for row in points],
               "o-", color=color, label=f"ground μ=μ2={mu}")
right.set(xlabel="Commanded slip (%)", ylabel="Fx / 24.5 N", title="Ground friction endpoints", xlim=(-2, 52))
right.legend(fontsize=8)
right.grid(alpha=0.2)
fig.savefig(here / "rigid_vs_paper.png", dpi=180)
