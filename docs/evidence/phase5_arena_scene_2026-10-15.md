# Guidebook arena-scene evidence — 2026-10-15

## Scope and source

This increment adds visualization-only static arena context without adding
navigation obstacles or transform authority. Values were transcribed from the
local 2026–2027 NASA Lunabotics Guidebook, Section 11.2 and Figure 9 (printed
pages 25–26): nominal arena `6.88 x 5.0 m`, excavation division `2.5 m`, and
target berm `2.2 x 0.9 m` centered at `(5.38, 0.9) m` from the starting-zone
arena-wall corner.

The Figure 9 starting-zone X extent is `2.0 m`; its Y depth is not numerically
specified. The rendered `2.0 m` depth is therefore dashed, configurable, and
recorded as an assumption. The central column is deliberately absent because
the guidebook provides no numeric pose or size.

## Components and interfaces

- `lb_navigation/scripts/arena_scene_markers.py` publishes a nine-marker
  `visualization_msgs/msg/MarkerArray` on
  `/visualization/arena/markers` with reliable, transient-local QoS and a 1 Hz
  reconnect/replay-clock heartbeat.
- `lb_navigation/config/arena_scene.yaml` is the single scene-geometry input.
- `lb_navigation/launch/arena_scene.launch.py` supports standalone display;
  `map.launch.py` starts it by default, and `arena_pose_replay.launch.py`
  starts exactly one copy independently of optional map-server availability.
- Both arena RViz profiles enable the scene namespaces.
- Static Tag 1 ownership moved from the expiring replay marker topic to the
  durable scene topic. Tags 2 and 3 are not rendered because they lack poses.

The scene contains a translucent light-yellow floor/outline, blue excavation
boundary, dashed green provisional starting zone, red berm target and center,
and orange assumed Tag 1 plane, face-normal arrow, and label. It contains no
column, boulder, crater, TF broadcaster, occupancy publisher, costmap input,
planner, controller, or actuation path.

## Assumption control

`docs/assumptions_register.md` records every currently identified temporary
hard number, provenance, consumer, prohibited use, and closure evidence. It
explicitly tracks the guidebook scene's `6.88 m` length versus the existing
cell-aligned occupancy interior's `6.85 m`, the inferred starting-zone depth,
tag/camera replay geometry, synthetic chassis/drive geometry, and replay
freshness setting. `docs/tbd_register.md` links to the numerical register.

## Verification

- Focused arena/replay unit and contract tests: 16 passed.
- Jazzy builds: `lb_localization`, `lb_navigation`, and `lb_sim` succeeded.
- Selected package suites: 126 tests, 0 errors, 0 failures, 1 pre-existing
  skip.
- Python syntax, `ament_flake8`, `ament_pep257`, CMake lint, XML lint, and
  `git diff --check` passed.
- Installed standalone launch published nine map-frame markers. Runtime output
  showed the `6.88 x 5.0 m` floor/outline, `x=2.5 m` division, dashed
  `2.0 x 2.0 m` provisional start, berm bounds
  `x=[4.28, 6.48], y=[0.45, 1.35]`, center `(5.38, 0.9)`, and Tag 1 at
  `[0, 1, 0.75]` with its normal toward `+X`. No column namespace was present.

Graphical RViz confirmation, arena survey, official/physical starting-zone
depth, tag survey, target Humble validation, and resource measurement remain
open. Scene markers must never be consumed as Nav2 collision geometry.

## Git evidence

Implementation is uncommitted on top of `e91be59` (`Phase 3 of mapping
process: localization from recorded apriltag bags`). Record the review commit
here and in the devlog after commit.
