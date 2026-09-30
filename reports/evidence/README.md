# Evidence Index

## Terrain v2 scene development

- [20260930T170028+0900](terrain-v2/20260930T170028+0900/DEVLOG.md): dated figures, research index, checksums, development log and validation boundary. The executable Gazebo assets remain in the ROS package's `worlds/`, `models/` and `materials/` directories.

## Delivery Runtime: de10bfa

- `mooncar-m6-survey-headless.{json,log}`: clean isolated headless run; same
  runtime and map content as de10bfa, with the freshly saved map passed by path.
- `mooncar-m6-survey-gui.{json,log}`: de10bfa with the default packaged map.
- `mooncar-m6-release-production.{json,log}`: third full run from the daily
  `/home/homura/MoonCar` clone with the default packaged map, headless.
- Matching `*-launch.log` files contain the ROS/Gazebo process output.
- `mooncar-survey-{gazebo,rviz}.png`: real GUI capture during/after that run.
- `mooncar-clean-survey-route.log`, `mooncar-clean-survey-truth.log`, and
  `mooncar-clean-map-save.log`: a8c9513 mapping run that produced the frozen map.
- `mooncar-survey-map.png`: enlarged observation map; authoritative pixels and
  metadata are in `ros2_ws/src/mooncar_navigation/maps/`.

## Superseded Experiments

- `mooncar-m6-production.log`: PR #51 production repeat failed at 0.52 m.
- `mooncar-m6-final-gui.*` and `mooncar-delivery-*.png`: 4de123d single GUI pass.
- `mooncar-m6-final-repeat.log`: 4de123d repeat failed at 0.53 m.
- `mooncar_m6_filtered_map.*` and `mooncar-m6-filtered-map.log`: short M4
  map used by 4de123d, replaced by the loop survey.
- Other `mooncar_m6_*`, `mooncar-m6-gui.log`, `mooncar-m6-mapping.log`, and
  `m6-*.png` files are the earlier PR #51 baseline, not current acceptance.

The word "final" in an older filename is historical, not a passing release
claim. Consult the delivery report for the accepted revision and repeat runs.
