# mooncar_terrain

Depth-based terrain observations, sparse keepout/traversability maps, map
recording and terrain-aware navigation helpers for MoonCar.

`mooncar_terrain::RiskOverlay` adds sparse risk grids to Nav2 costmaps without
clearing other sources. Unknown/free cells leave the master untouched; positive
costs replace unknown or increase existing risk. It resamples grids through the
map-to-master transform and uses layer-specific trinary/continuous settings.
Rolling-window bounds cover the master window in its own frame.

Build the package before starting terrain navigation so pluginlib can discover
the overlay. Native tests cover unknown/free merging and grids with different
origins and resolutions; the end-to-end navigation evidence checks that known
keepout zones remain present in the actual global costmap.
