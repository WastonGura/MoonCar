"""Render the measured x-position response from the frozen sweep."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


HERE = Path(__file__).resolve().parent
COLORS = {"scale_025": "#C15A37", "scale_050": "#C69A36",
          "scale_075": "#347F9D", "scale_100": "#344E83"}

fig, ax = plt.subplots(figsize=(8, 4.5), layout="constrained")
for case, color in COLORS.items():
    rows = [json.loads(line) for line in (HERE / f"{case}_samples.jsonl").read_text().splitlines()]
    t0, x0 = rows[0]["sim_seconds"], rows[0]["x"]
    ax.plot([row["sim_seconds"] - t0 for row in rows], [row["x"] - x0 for row in rows],
            marker="o", markersize=3, linewidth=2, color=color, label=f"{int(case[-3:])}%")
ax.set(title="R5 ramp: ground friction scale response", xlabel="Driving time (sim s)",
       ylabel="Forward displacement (m)", xlim=(0, 20), ylim=(0, 5))
ax.grid(alpha=0.18)
ax.legend(title="Baseline fraction", frameon=False)
fig.savefig(HERE / "progress_vs_time.png", dpi=180)
