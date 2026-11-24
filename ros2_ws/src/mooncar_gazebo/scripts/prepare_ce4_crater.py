#!/usr/bin/env python3
"""Downsample the attributed Chang'e-4 crater DTM for the terrain generator.

Requires scipy only for this one-time conversion of the original MATLAB file.
The resulting NPZ is read by build_terrain_v2.py with NumPy alone.
"""

import hashlib
from pathlib import Path

import numpy as np
from scipy.io import loadmat
from scipy.ndimage import gaussian_filter


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "data/terrain_sources/CE-4_Fresh_Crater_DEM.mat"
OUTPUT = ROOT / "data/terrain_sources/ce4_fresh_crater_5cm.npz"
SOURCE_MD5 = "39a21d550aa9376c2497b26324bb76b9"
ROW0, COL0, SAMPLES, STRIDE = 400, 150, 961, 10


def main():
    assert hashlib.md5(SOURCE.read_bytes()).hexdigest() == SOURCE_MD5
    dem = loadmat(SOURCE)["fresh_crater_dem"]
    crop = dem[ROW0:ROW0 + SAMPLES, COL0:COL0 + SAMPLES]
    assert crop.shape == (SAMPLES, SAMPLES) and np.all(crop > -50)
    # The paper's north-up image has row 0 at the northern edge.
    sampled = np.flipud(gaussian_filter(crop, sigma=5.0)[::STRIDE, ::STRIDE])
    coords = np.linspace(-2.4, 2.4, sampled.shape[0])
    yy, xx = np.meshgrid(coords, coords, indexing="ij")
    edge = (np.abs(xx) >= 2.1) | (np.abs(yy) >= 2.1)
    plane = np.linalg.lstsq(
        np.column_stack((np.ones(edge.sum()), xx[edge], yy[edge])),
        sampled[edge], rcond=None,
    )[0]
    residual = sampled - (plane[0] + plane[1] * xx + plane[2] * yy)
    assert residual.shape == (97, 97)
    assert 0.30 < float(residual.max() - residual.min()) < 0.45
    np.savez_compressed(OUTPUT, x=coords, y=coords, z=residual,
                        source_md5=SOURCE_MD5, row0=ROW0, col0=COL0,
                        source_step_m=0.005, output_step_m=0.05,
                        removed_plane=plane)
    print(f"{OUTPUT}: residual [{residual.min():.3f}, {residual.max():.3f}] m")


if __name__ == "__main__":
    main()
