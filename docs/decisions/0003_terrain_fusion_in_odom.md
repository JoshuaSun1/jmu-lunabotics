# ADR 0003: Fuse depth terrain over time in the odom frame

**Date:** 2026-11-18
**Status:** Accepted; target validation open
**Owner:** JMU Lunabotics software team

## Context

The first `terrain_hazard_node` judged every ZED depth frame on its own in
`base_link`. Comparing neighboring 5 cm cells turned 1–2 cm of stereo noise
into slope hazards, and a crater whose floor is hidden by its near rim showed
up only as missing depth, which the node reported as a warning but never as a
hazard. The spec (section 8.2, step 7) asks for negative obstacles from
missing-ground evidence.

## Options

1. Keep single-frame detection in `base_link` and raise thresholds. Simple,
   but it trades noise for missed rocks and never sees a hidden crater floor.
2. Average frames in `base_link`. Wrong as soon as the robot moves.
3. Average frames in `odom` and infer hidden drops from line-of-sight
   geometry. Needs continuous local odometry.
4. Adopt an elevation-mapping library (`grid_map`, `elevation_mapping_cupy`).
   Heavier, with uncertain Jazzy/JetPack 7 support.

## Decision

Option 3. Each frame's cell heights, measured relative to that frame's own
ground plane, are averaged into a local map in `fusion_frame` (default
`odom`). Cells are classified only after repeated observations and only when
they clear a threshold by a set number of standard errors. A depth gap is a
hidden drop only when the sight line over the nearer terrain passes below the
drop threshold and the far side of the gap is measured at or below that line.
Both clouds are stamped in `fusion_frame`, not `base_link`.

## Consequences

- The default mode needs an `odom -> base_link` TF, which waits for Phase 3
  local odometry. Until then the node must run with `fusion_frame: base_link`,
  `min_observations: 1`, and `min_hidden_frames: 1` (single-frame, no
  averaging or memory); it refuses `base_link` with multi-frame settings.
- Hazards persist after leaving the camera's view until `cell_memory_s` or
  `map_radius_m` expires them, which bounds odometry drift.
- Spec section 8.2 step 4 ("transform samples to `base_link`") now describes
  only the single-frame fallback.
- A future costmap consumer must accept clouds in `odom`.

## Validation

Synthetic gtests cover noise suppression, observation counts, out-of-view
memory and expiry, timestamp rewind, hidden crater floors, stereo dropouts,
and rock shadows. Not yet tested: a ROS 2 Jazzy build of the node, real ZED
data, Jetson performance, and threshold tuning against recorded arena bags.
See [the ZED Mini guide](../zed_mini.md#terrain-hazard-detector).
