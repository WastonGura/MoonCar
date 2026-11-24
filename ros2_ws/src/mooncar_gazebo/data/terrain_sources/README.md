# Chang'e-4 fresh-crater elevation source

`CE-4_Fresh_Crater_DEM.mat` is the original 5 mm/pixel DTM published by
Chunyu Ding, Zhiyong Xiao, Bo Wu, Yuan Li, Nils C. Prieur, Yuzhen Cai,
Yan Su and Jun Cui for *Fragments Delivered by Secondary Craters at the
Chang'E-4 Landing Site*, Geophysical Research Letters 47 (2020),
https://doi.org/10.1029/2020GL087361.

Source record: https://doi.org/10.5281/zenodo.3732816

Source license: CC BY 4.0, https://creativecommons.org/licenses/by/4.0/

Published MD5: `39a21d550aa9376c2497b26324bb76b9`.

`ce4_fresh_crater_5cm.npz` is a modified derivative made for MoonCar by
`scripts/prepare_ce4_crater.py`: valid 4.8 m crop, 5 cm sampling, north-up
orientation and border-plane removal. The scene generator feathers its outer
0.35 m into synthetic terrain. Neither the derivative nor the surrounding
world is an unmodified georeferenced DEM.

## Wider Chang'e-4 DTM and valid background crop

`CE4_Day_9_10_DEM_5mm.tif` is kept in the local dated research bundle
`reports/evidence/terrain-v2/20260930T170028+0900/sources/`, not in the ROS
package or Git. It is the original 5 mm/pixel DTM published with
Dijun Guo, Wenzhe Fa, Bo Wu, Yuan Li and Yang Liu, *Millimeter- to
Decimeter-Scale Surface Roughness of the Moon at the Chang'e-4 Exploration
Region*, Geophysical Research Letters 48 (2021),
https://doi.org/10.1029/2021GL094931. The data record is
https://doi.org/10.5281/zenodo.4971121 (CC BY 4.0); its published MD5 is
`985ec0b225279df836cba87a5ecd306b`, verified after download.
The record also credits Bo Wu et al., *Characterization of the Chang'E-4
Landing Site and Exploration Region Using Descent and Rover Images*,
Earth and Planetary Science Letters 553 (2021),
https://doi.org/10.1016/j.epsl.2020.116666. These are reconstructed lunar
elevations from imagery; the 5 mm pixel spacing is not a 5 mm accuracy claim.

The 3738 × 4096 TIFF has 5 mm pixels and a `-9999` no-data value; 69.4727%
of pixels are valid. The original full raster remains outside the ROS package.
`scripts/prepare_ce4_background.py <path-to-original-tif>` verifies its published
MD5 and produces `ce4_background_5cm.npz` from rows 2575–3775 and columns
1261–2461 (zero-based, inclusive), a fully valid 6 m square. This crop is centered
inside the largest all-valid axis-aligned square; selection uses missing-data
coverage, not rover results. It preserves north-up orientation and horizontal/
vertical scale, applies a Gaussian low-pass with sigma=5 original pixels,
samples at 5 cm, and subtracts the center height (-1.8710692848 m). Regional
slope is retained; no plane detrending, no filling and no tiling is performed.

The generator places this derivative at (6.0,-5.1) m in `survey`, replacing
synthetic relief inside a 6 m square. The outer 0.35 m blends into the actual
coarse mesh; the 5.3 m interior uses the derivative alone. Placement, blending,
material labels and 5 cm sampling are engineering choices. This is local
measured geometry, not a full Chang'e-4 reconstruction or proof of 125 mm-window
millimeter roughness. The other source crater patch remains unchanged.
