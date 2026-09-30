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

## Wider Chang'e-4 DTM (reference only)

`CE4_Day_9_10_DEM_5mm.tif` is kept in the local dated research bundle
`reports/evidence/terrain-v2/20260930T170028+0900/sources/`, not in the ROS
package or Git. It is the original 5 mm/pixel DTM published with
Dijun Guo, Wenzhe Fa, Bo Wu, Yuan Li and Yang Liu, *Millimeter- to
Decimeter-Scale Surface Roughness of the Moon at the Chang'e-4 Exploration
Region*, Geophysical Research Letters 48 (2021),
https://doi.org/10.1029/2021GL094931. The data record is
https://doi.org/10.5281/zenodo.4971121 (CC BY 4.0); its published MD5 is
`985ec0b225279df836cba87a5ecd306b`, verified after download.

The 3738 × 4096 TIFF has 5 mm pixels and a `-9999` no-data value; only about
69.5% of its pixels are valid, including a large central void. It is retained
for parameter checks and possible later scene design, not embedded in the
current worlds. Filling the void and making a single continuous drivable mesh
would require a separately documented reconstruction method.
