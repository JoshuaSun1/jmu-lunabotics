# Static arena map-server evidence — 2026-10-11

## Scope

This increment configures lifecycle-managed publication of the previously
reviewed static arena artifact. It adds no robot pose, localization TF,
costmap, planner, controller, behavior tree, sensor, or actuation path.

## Components and interfaces

- `lb_navigation/config/map_server.yaml` configures `map_server` in frame
  `map` and `lifecycle_manager_map` with autostart and only `map_server` in its
  managed-node list.
- `lb_navigation/launch/map.launch.py` validates explicit files, supplies the
  installed arena YAML, starts `nav2_map_server/map_server`, starts
  `nav2_lifecycle_manager/lifecycle_manager`, and optionally starts RViz.
- `lb_navigation/rviz/arena_map.rviz` uses fixed frame `map`, a top-down view,
  and transient-local reliable QoS for `/map`.
- Expected public output is `/map` as `nav_msgs/msg/OccupancyGrid` with
  `header.frame_id=map`; standard Nav2 load-map and lifecycle services are
  exposed by the upstream nodes. No TF is published.

## Parameters and provenance

The static artifact remains 139 x 102 cells at 0.05 m/cell, with a 137 x 100
free interior representing 6.85 m x 5.00 m. The YAML image origin remains
`[-0.05, -0.05, 0.0]`. The lifecycle manager defaults to autostart; simulated
time defaults off. The 6.85 m length is a team-selected working value, the
5.00 m width comes from the guidebook, and the center column remains unmeasured
and omitted.

## Verification

- Focused arena/map-server pytest: 6 passed.
- `lb_navigation` Jazzy build completed and installed configuration, launch,
  maps, and RViz profiles.
- Installed launch `--show-args` resolved all installed default paths.
- `lb_navigation` colcon test: 8 tests, 0 errors, 0 failures, 0 skipped.
- New Python launch/tests passed `ament_flake8` and `ament_pep257`.
- `git diff --check` passed before documentation finalization.

Runtime lifecycle and `/map` topic validation could not be completed on this
host because `nav2_map_server` and `nav2_lifecycle_manager` are not installed.
APT reports Jazzy 1.3.12 candidates for both. The attempted minimal install was
stopped by the host's interactive sudo-password requirement; no system package
was installed or changed. Launch therefore fails explicitly with
`package 'nav2_map_server' not found` rather than starting a partial graph.

After installing declared dependencies, perform:

```bash
source /opt/ros/jazzy/setup.bash
source /home/jsun3/dev_ws/install/setup.bash
ros2 launch lb_navigation map.launch.py
```

Then verify the lifecycle state, topic durability, frame, resolution,
dimensions, origin, occupied border, and visual alignment using the commands in
`docs/operations.md`.

## Safety and remaining work

The launch is non-actuating and contains no transform publisher. `map_server`
publishes an occupancy grid, not `map -> odom`; the future global EKF retains
that authority. Required next work is the runtime/GUI smoke test, target Humble
validation, physical arena control-point verification, and later survey of the
center column. Robot-pose replay remains a separate next increment.

Implementation commit: `69b0505` (`In the middle of stage 2 of map
implementation`). The 2026-10-11 documentation/readiness updates remain
uncommitted.
