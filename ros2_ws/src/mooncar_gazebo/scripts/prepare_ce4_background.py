#!/usr/bin/env python3
"""Crop valid Guo 2021 DTM at original scale, without filling no-data."""
import hashlib
from pathlib import Path
import sys
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "data/terrain_sources/ce4_background_5cm.npz"
SOURCE_MD5 = "985ec0b225279df836cba87a5ecd306b"
# Centered 6 m crop in the largest all-valid square; selection by validity only.
ROW0, COL0, SAMPLES, STRIDE = 2575, 1261, 1201, 10


def main(source):
    assert hashlib.md5(source.read_bytes()).hexdigest() == SOURCE_MD5
    with Image.open(source) as image:
        assert image.tag_v2[33550][:2] == (0.005, 0.005)
        dem = np.asarray(image)
    crop = dem[ROW0:ROW0 + SAMPLES, COL0:COL0 + SAMPLES]
    assert crop.shape == (SAMPLES, SAMPLES)
    assert np.all(np.isfinite(crop) & (crop != -9999))
    sampled = np.flipud(gaussian_filter(crop.astype(float), sigma=5.0)[::STRIDE, ::STRIDE])
    coords = np.linspace(-3.0, 3.0, sampled.shape[0])
    datum = float(sampled[60, 60])  # Translation only; keep slope and height scale.
    np.savez_compressed(OUTPUT, x=coords, y=coords, z=sampled - datum,
                        source_md5=SOURCE_MD5, row0=ROW0, col0=COL0,
                        source_step_m=0.005, output_step_m=0.05,
                        removed_height_datum_m=datum)
    print(f"{OUTPUT}: 6 m crop, no-data 0, datum {datum:.6f} m")


if __name__ == "__main__":
    main(Path(sys.argv[1]))
