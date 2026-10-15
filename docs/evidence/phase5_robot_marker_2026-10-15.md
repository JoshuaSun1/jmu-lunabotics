# Development Robot Marker Evidence — 2026-10-15

## Scope

Prompt 4 extends the provisional `test1_AT1` arena replay with a recognizable
robot proxy. It does not create a URDF robot state, publish TF, localize through
tag loss, or provide collision geometry to Nav2.

## Implemented geometry and provenance

- Synthetic chassis: `0.90 x 0.60 x 0.25 m`, reused from
  `lb_model/config/mock_geometry.yaml`; synthetic and unmeasured.
- Footprint: closed rectangle matching only the synthetic chassis; it excludes
  the scoop and must not be used as the production Nav2 footprint.
- Rear webcam: `[0, 0, 0.40] m`, optical axis toward robot `-X`, pink.
- Forward ZED: `[0, 0, 0.40] m`, optical axis toward robot `+X`, green.
- Camera locations/orientations are user-selected development assumptions.
  The overlapping bodies are distinguished by color and direction arrows.

All dynamic markers use a 0.5 s lifetime and are explicitly deleted when the
tag observation becomes stale. The node continues to publish no TF, preserving
future EKF ownership of `map -> odom` and `odom -> base_link`.

## Verification

- Focused Python tests: 8 passed. These cover rigid-transform recovery,
  planarization, loss/reset segmentation, the closed rotated footprint, and
  rear/forward optical-axis conventions.
- `ament_flake8` and `ament_pep257`: no problems found.
- Jazzy `colcon build --packages-select lb_localization lb_navigation lb_sim`:
  3 packages built successfully.
- Selected package suites: 120 tests, 0 errors, 0 failures, 1 pre-existing
  skip.
- Live headless `test1_AT1` replay produced an 11-marker `MarkerArray` in
  `map`. The observed chassis scale was `(0.9, 0.6, 0.25)`; both sensor bodies
  were at `z=0.4`; the pink arrow opposed and the green arrow followed the
  planar robot heading. The sample footprint contained five points, closing
  its rectangle.

Graphical RViz inspection, physical dimensions/mount survey, target Humble
validation, and resource measurements remain open. The static map was disabled
for this headless marker check; earlier map-server dependency limitations are
unchanged.

## Git evidence

Implementation is uncommitted on top of `e91be59` (`Phase 3 of mapping
process: localization from recorded apriltag bags`). Record the review commit
here and in the devlog after commit.
