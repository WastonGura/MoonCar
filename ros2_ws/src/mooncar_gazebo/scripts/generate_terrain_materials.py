#!/usr/bin/env python3
"""
Generate procedurally-noised PBR terrain textures for the MoonCar Gazebo world.

Why this exists
---------------
The original `mooncar_gazebo/worlds/lunar_yard.sdf` used flat colours. These
maps add reproducible visual texture while avoiding obvious colour-coded
class labels. They are for simulation and are not lunar spectral measurements.

This script generates, per terrain class, three seamless tileable maps:

    <class>_albedo.png     RGB  base colour + per-class noise structure
    <class>_normal.png     RGB  tangent-space normal, derived from a height field
    <class>_roughness.png  L    micro-roughness variation

The classes share a restrained grey-brown palette. The maps are procedural
visual hypotheses, not calibrated lunar spectral or soil measurements.

Seamlessness: the noise is synthesised in the frequency domain with random
phase, so every map is intrinsically periodic and tiles without visible seams.

Output (default):
    ros2_ws/src/mooncar_gazebo/materials/textures/*.png
    ros2_ws/src/mooncar_gazebo/materials/preview/terrain_materials_preview.png

Dependencies: numpy, pillow.

    python3 generate_terrain_materials.py [--size 512] [--out DIR] [--verify-only]
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image

DEFAULT_SIZE = 512
SEED = 20260928

# ---------------------------------------------------------------------------
# Class definitions
#
# base       : linear-ish sRGB base colour; close hues avoid a class-colour key
# contrast   : albedo brightness modulation depth
# alpha      : spectral slope of the albedo noise (small = grainy, large = smooth)
# chroma     : 0 = use one shared noise for RGB (grey), 1 = per-channel noise
# rough      : (mean, variation) of the roughness map
# normal     : normal-map strength (higher = more relief)
# aniso      : (amount, angle_rad) directional banding, used for rock bedding
# blocky     : >0 pushes the grain toward discrete pebbles / clasts
# ---------------------------------------------------------------------------
CLASSES: dict[str, dict] = {
    "regolith_passable": dict(
        # medium-light neutral grey, fine uniform grain
        base=(0.48, 0.47, 0.45),
        contrast=0.070,
        alpha=1.25,
        chroma=0.30,
        rough=(0.86, 0.06),
        normal=1.5,
        aniso=(0.0, 0.0),
        blocky=0.0,
    ),
    "sand_soft": dict(
        # modestly lighter fine-grained region, without a yellow class cue
        base=(0.51, 0.49, 0.46),
        contrast=0.055,
        alpha=2.05,
        chroma=0.55,
        rough=(0.96, 0.03),
        normal=0.9,
        aniso=(0.0, 0.0),
        blocky=0.0,
    ),
    "rock_boulder": dict(
        # angular rock, distinguished mostly by texture and silhouette
        base=(0.39, 0.385, 0.38),
        contrast=0.170,
        alpha=1.45,
        chroma=0.15,
        rough=(0.58, 0.14),
        normal=2.6,
        aniso=(0.55, 0.6),
        blocky=0.0,
    ),
    "rubble_gravel": dict(
        # mid grey, high variance, discretely pebbled
        base=(0.45, 0.44, 0.43),
        contrast=0.260,
        alpha=0.70,
        chroma=0.40,
        rough=(0.92, 0.08),
        normal=2.2,
        aniso=(0.0, 0.0),
        blocky=1.0,
    ),
    "crater_floor": dict(
        # floor darkness should come from scene illumination, not a painted mask
        base=(0.47, 0.46, 0.44),
        contrast=0.110,
        alpha=2.40,
        chroma=0.20,
        rough=(0.90, 0.05),
        normal=1.1,
        aniso=(0.0, 0.0),
        blocky=0.0,
    ),
}


def spectral_noise(
    size: int,
    rng: np.random.Generator,
    alpha: float,
    aniso_amount: float = 0.0,
    aniso_angle: float = 0.0,
) -> np.ndarray:
    """Periodic, zero-mean, unit-variance noise with a 1/f**alpha spectrum.

    Synthesised directly in the frequency domain, so the result is seamless by
    construction. `alpha` controls the slope: small values keep more high
    frequency content (grainy), large values are smoother (broad drifts).
    """
    fy = np.fft.fftfreq(size)[:, None]
    fx = np.fft.fftfreq(size)[None, :]
    freq = np.sqrt(fx**2 + fy**2)
    # avoid 1/0 at DC; scale is irrelevant because we renormalise afterwards
    freq[0, 0] = freq[0, 1]

    amp = freq ** (-alpha)

    if aniso_amount > 0.0:
        ang = np.arctan2(fy, fx)
        # suppress energy along one direction -> bedding / banding
        amp = amp * (1.0 - aniso_amount * np.abs(np.cos(ang - aniso_angle)) ** 4)

    amp[0, 0] = 0.0
    spec = amp * np.exp(1j * rng.uniform(0.0, 2.0 * np.pi, (size, size)))
    img = np.fft.ifft2(spec).real

    img -= img.mean()
    std = img.std()
    if std <= 1e-12:
        raise RuntimeError("degenerate noise field")
    return img / std


def apply_blocky(noise: np.ndarray, amount: float) -> np.ndarray:
    """Push smooth grain toward discrete clasts (pebbles) without going binary."""
    if amount <= 0.0:
        return noise
    sharpened = np.tanh(2.4 * noise)          # saturate the tails
    blended = (1.0 - amount) * noise + amount * sharpened * 1.15
    return blended


def height_to_normal(height: np.ndarray, strength: float) -> np.ndarray:
    """Tangent-space normal map (RGB, 0..255) from a periodic height field."""
    gy, gx = np.gradient(height)
    nx = -gx * strength
    ny = -gy * strength
    nz = np.ones_like(height)
    norm = np.sqrt(nx**2 + ny**2 + nz**2)
    nx, ny, nz = nx / norm, ny / norm, nz / norm
    rgb = np.stack([nx, ny, nz], axis=-1)
    return np.clip((rgb * 0.5 + 0.5) * 255.0, 0, 255).astype(np.uint8)


def build_class(name: str, cfg: dict, size: int, seed: int) -> dict[str, np.ndarray]:
    rng = np.random.default_rng(seed)
    a = cfg["alpha"]
    aniso_amt, aniso_ang = cfg["aniso"]

    shared = spectral_noise(size, rng, a, aniso_amt, aniso_ang)
    shared = apply_blocky(shared, cfg["blocky"])

    base = np.array(cfg["base"], dtype=np.float64)
    contrast = cfg["contrast"]
    chroma = cfg["chroma"]

    channels = []
    for _ in range(3):
        per_ch = spectral_noise(size, rng, a * 0.9, aniso_amt, aniso_ang)
        per_ch = apply_blocky(per_ch, cfg["blocky"])
        field = (1.0 - chroma) * shared + chroma * per_ch
        channels.append(field)

    field_rgb = np.stack(channels, axis=-1)
    albedo = base[None, None, :] * (1.0 + contrast * field_rgb)
    albedo = np.clip(albedo, 0.015, 0.985)
    albedo_u8 = (albedo * 255.0).round().astype(np.uint8)

    # normals come from a slightly finer height field than the albedo structure
    height = spectral_noise(size, rng, max(a * 0.65, 0.35), aniso_amt, aniso_ang)
    height = apply_blocky(height, cfg["blocky"] * 0.6)
    normal_u8 = height_to_normal(height, cfg["normal"])

    r_mean, r_var = cfg["rough"]
    r_noise = spectral_noise(size, rng, 1.1)
    rough = np.clip(r_mean + r_var * r_noise, 0.03, 1.0)
    rough_u8 = (rough * 255.0).round().astype(np.uint8)

    return {"albedo": albedo_u8, "normal": normal_u8, "roughness": rough_u8}


# ---------------------------------------------------------------------------
# Verification
# ---------------------------------------------------------------------------
def seam_score(img: np.ndarray) -> float:
    """Mean |gradient| across the wrap edge divided by the mean interior one.

    A seamless tile scores ~1.0; a tile with a visible seam scores well above 1.
    """
    g = img.astype(np.float64)
    if g.ndim == 3:
        g = g.mean(axis=-1)
    edge = np.abs(g[:, 0] - g[:, -1]).mean() + np.abs(g[0, :] - g[-1, :]).mean()
    interior = np.abs(np.diff(g, axis=1)).mean() + np.abs(np.diff(g, axis=0)).mean()
    return float(edge / (2.0 * interior)) if interior > 0 else float("nan")


def mean_srgb(img: np.ndarray) -> tuple[float, ...]:
    g = img.astype(np.float64)
    if g.ndim == 2:
        g = np.stack([g] * 3, axis=-1)
    m = g.reshape(-1, 3).mean(axis=0)
    return tuple(float(v) for v in m)


def build_preview(maps: dict[str, dict[str, np.ndarray]], size: int) -> Image.Image:
    """One row per class: albedo | normal | roughness."""
    order = list(maps.keys())
    rows = []
    for name in order:
        m = maps[name]
        rough_rgb = np.stack([m["roughness"]] * 3, axis=-1)
        rows.append(np.concatenate([m["albedo"], m["normal"], rough_rgb], axis=1))
    grid = np.concatenate(rows, axis=0)
    return Image.fromarray(grid, "RGB")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", type=int, default=DEFAULT_SIZE)
    ap.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "materials",
    )
    ap.add_argument(
        "--verify-only",
        action="store_true",
        help="re-check textures already on disk instead of regenerating them",
    )
    args = ap.parse_args()

    tex_dir = args.out / "textures"
    prev_dir = args.out / "preview"
    tex_dir.mkdir(parents=True, exist_ok=True)
    prev_dir.mkdir(parents=True, exist_ok=True)

    maps: dict[str, dict[str, np.ndarray]] = {}

    if args.verify_only:
        for name in CLASSES:
            maps[name] = {
                kind: np.array(Image.open(tex_dir / f"{name}_{kind}.png"))
                for kind in ("albedo", "normal", "roughness")
            }
    else:
        for i, (name, cfg) in enumerate(CLASSES.items()):
            maps[name] = build_class(name, cfg, args.size, SEED + 1000 * i)
            for kind, arr in maps[name].items():
                mode = "L" if kind == "roughness" else "RGB"
                Image.fromarray(arr, mode).save(tex_dir / f"{name}_{kind}.png", optimize=True)

    # ---- report -----------------------------------------------------------
    report: dict = {"size": args.size, "classes": {}, "pairwise": {}, "seams": {}}
    print(f"texture size : {args.size}x{args.size}")
    print(f"output dir   : {tex_dir}")
    print()
    print(f"{'class':<20}{'mean sRGB (0-255)':<26}{'lum':>7}{'seam':>8}")
    print("-" * 62)

    lum: dict[str, float] = {}
    for name, m in maps.items():
        mr, mg, mb = mean_srgb(m["albedo"])
        # perceptual luminance (Rec.709) in 0-255
        l = 0.2126 * mr + 0.7152 * mg + 0.0722 * mb
        lum[name] = l
        seam = seam_score(m["albedo"])
        report["classes"][name] = {
            "mean_srgb": [round(mr, 1), round(mg, 1), round(mb, 1)],
            "luminance": round(l, 1),
            "seam_ratio": round(seam, 3),
        }
        report["seams"][name] = round(seam, 3)
        print(f"{name:<20}{f'({mr:5.1f},{mg:5.1f},{mb:5.1f})':<26}{l:7.1f}{seam:8.3f}")

    print()
    print("pairwise mean-colour distance (RGB euclidean, 0-255 scale):")
    names = list(maps.keys())
    print(f"{'':<20}" + "".join(f"{n[:9]:>11}" for n in names))
    worst = 1e9
    for a in names:
        ma = np.array([mean_srgb(maps[a]["albedo"])])
        row = []
        for b in names:
            mb = np.array([mean_srgb(maps[b]["albedo"])])
            d = float(np.linalg.norm(ma - mb))
            row.append(d)
            if a != b:
                worst = min(worst, d)
            report["pairwise"][f"{a}|{b}"] = round(d, 2)
        print(f"{a:<20}" + "".join(f"{v:11.1f}" for v in row))

    print()
    print(f"closest pair distance : {worst:.1f} / 255")
    report["closest_pair_distance"] = round(worst, 2)
    report["palette_status"] = "procedural, not spectrally calibrated"

    preview = build_preview(maps, args.size)
    preview_path = prev_dir / "terrain_materials_preview.png"
    preview.save(preview_path, optimize=True)
    print(f"preview (rows=classes, cols=albedo|normal|roughness): {preview_path}")

    report_path = args.out / "terrain_materials_report.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"report: {report_path}")

    total = sum(p.stat().st_size for p in tex_dir.glob("*.png"))
    print(f"textures on disk: {len(list(tex_dir.glob('*.png')))} files, {total/1e6:.1f} MB")

    unused = args.size  # keep linters quiet about unused param usage
    del unused
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
