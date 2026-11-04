#!/usr/bin/env python3
"""Render a plan view and two marked terrain sections for the three v2 worlds."""

import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap, TwoSlopeNorm
from matplotlib.patches import Circle, Rectangle
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
SECTIONS = {"ramp": (0.0, 4.0), "curve": (0.0, 4.0), "survey": (3.6, 4.8)}
CMAP = LinearSegmentedColormap.from_list("lunar_relief", ["#474955", "#98958e", "#d2bd92"])


def draw():
    fig, axes = plt.subplots(3, 3, figsize=(15, 11), layout="constrained")
    fig.patch.set_facecolor("#f4f4f2")
    heights = [np.load(ROOT / "models" / "terrain_v2" / scene / "height_grid.npz")["z"]
               for scene in SECTIONS]
    norm = TwoSlopeNorm(vmin=min(-0.08, *(float(z.min()) for z in heights)),
                        vcenter=0, vmax=max(0.08, *(float(z.max()) for z in heights)))
    for row, (scene, (section_y, section_x)) in enumerate(SECTIONS.items()):
        folder = ROOT / "models" / "terrain_v2" / scene
        data = np.load(folder / "height_grid.npz")
        xs, ys, z = data["x"], data["y"], data["z"]
        manifest = json.loads((folder / "manifest.json").read_text(encoding="utf-8"))
        plan, side, front = axes[row]
        view = plan.imshow(z, origin="lower", extent=(xs[0], xs[-1], ys[0], ys[-1]),
                           cmap=CMAP, norm=norm)
        plan.contour(xs, ys, z, levels=7, colors="#292b31", linewidths=0.4, alpha=0.30)
        for rock in manifest["boulders"]:
            x, y, _ = rock["position_m"]
            rx, ry, _ = rock["radii_m"]
            plan.add_patch(Circle((x, y), max(rx, ry), facecolor="#30333a",
                                  edgecolor="#edece8", linewidth=1.1))
        for key in ("measured_patch", "measured_background_patch"):
            if not manifest.get(key):
                continue
            px, py = manifest[key]["center_m"]
            width = manifest[key]["size_m"]
            plan.add_patch(Rectangle((px - width / 2, py - width / 2), width, width,
                                     fill=False, edgecolor="#d48640", linewidth=1.4))
            plan.text(px + width / 2 + 0.3, py + width / 2,
                      "CE-4 measured patch", color="#aa6128", fontsize=9, va="top")
        plan.axhline(section_y, color="#37c1cf", linestyle="--", linewidth=1.7)
        plan.axvline(section_x, color="#e69c4c", linestyle="--", linewidth=1.7)
        plan.set(xlabel="x (m)", ylabel=f"{scene.upper()}   y (m)")
        plan.set_aspect("equal")

        x_profile = np.array([np.interp(section_y, ys, z[:, col]) for col in range(len(xs))])
        y_profile = np.array([np.interp(section_x, xs, z[r]) for r in range(len(ys))])
        low, high = min(-0.08, float(z.min()) - 0.05), max(0.08, float(z.max()) + 0.05)
        for ax, coord, profile, color, label in (
            (side, xs, x_profile, "#278d9a", f"y = {section_y:g} m"),
            (front, ys, y_profile, "#c88035", f"x = {section_x:g} m"),
        ):
            ax.plot(coord, profile, color=color, linewidth=2.3)
            ax.fill_between(coord, low, profile, color=color, alpha=0.14)
            ax.axhline(0, color="#646a70", linewidth=0.8, linestyle=":")
            ax.set(xlim=(coord[0], coord[-1]), ylim=(low, high), xlabel="x (m)" if ax is side else "y (m)",
                   ylabel="height (m)")
            ax.text(0.97, 0.90, label, transform=ax.transAxes, ha="right", va="top", color=color)
            ax.grid(color="#d9d9d6", linewidth=0.6)
        if row == 0:
            for ax, title in zip((plan, side, front), ("TOP VIEW", "X–Z SECTION", "Y–Z SECTION")):
                ax.set_title(title, weight="bold", fontsize=13)
    fig.colorbar(view, ax=axes[:, 0], label="relative elevation (m)")
    fig.suptitle("MoonCar terrain v2  |  top view and marked sections", fontsize=17, weight="bold")
    output = ROOT / "docs" / "terrain_v2_three_views.png"
    output.parent.mkdir(exist_ok=True)
    fig.savefig(output, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(output)


def draw_measured_patch():
    with np.load(ROOT / "data/terrain_sources/ce4_fresh_crater_5cm.npz") as data:
        x, y, z = data["x"], data["y"], data["z"]
    row, col = np.unravel_index(np.argmin(z), z.shape)
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8), layout="constrained")
    fig.patch.set_facecolor("#f4f4f2")
    plan, east_west, north_south = axes
    view = plan.imshow(z, origin="lower", extent=(x[0], x[-1], y[0], y[-1]),
                       cmap=CMAP, norm=TwoSlopeNorm(vmin=-0.25, vcenter=0, vmax=0.14))
    plan.contour(x, y, z, levels=12, colors="#292b31", linewidths=0.4, alpha=0.4)
    plan.axhline(y[row], color="#37c1cf", linestyle="--")
    plan.axvline(x[col], color="#e69c4c", linestyle="--")
    plan.set(xlabel="local x (m)", ylabel="local y (m)", title="MEASURED PATCH")
    plan.set_aspect("equal")
    fig.colorbar(view, ax=plan, label="height after border-plane removal (m)")
    for ax, coords, heights, color, title in (
        (east_west, x, z[row], "#278d9a", "E–W THROUGH LOW POINT"),
        (north_south, y, z[:, col], "#c88035", "N–S THROUGH LOW POINT"),
    ):
        ax.plot(coords, heights, color=color, linewidth=2)
        ax.axhline(0, color="#646a70", linewidth=0.8, linestyle=":")
        ax.set(xlabel="local distance (m)", ylabel="height (m)", title=title,
               ylim=(-0.27, 0.15))
        ax.grid(color="#d9d9d6", linewidth=0.6)
    fig.suptitle("Chang'e-4 fresh crater DTM derivative | 5 cm grid, no scale change",
                 fontsize=13, weight="bold")
    output = ROOT / "docs/ce4_measured_patch_detail.png"
    fig.savefig(output, dpi=150, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(output)


if __name__ == "__main__":
    draw()
    draw_measured_patch()
