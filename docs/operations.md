# Operations

## Profiles

- **Development host:** may run static checks and mock/simulation work. It does
  not validate the Jetson runtime lock.
- **Jetson bench:** requires the locked JetPack/L4T, Humble, ZED SDK, and
  wrapper versions to be verified on the physical target before sensor work.
- **Competition:** headless runtime; motors disabled by default and RViz run
  offboard where possible.

## Safe setup

1. Review `docs/resources/software_platform_lock.md`. The robot runtime is Ubuntu
   22.04 / ROS 2 Humble, not the development workstation's Jazzy override.
2. Provision each supported device once from the repository root:

   ```bash
   # Ubuntu 22.04 (Jammy) development host
   ./scripts/bootstrap_dev.sh --install --profile development

   # Only after the locked JetPack / L4T image is manually verified on Orin
   ./scripts/bootstrap_dev.sh --install --profile jetson
   ```

   The installer is explicit; its normal default, `--check`, is non-mutating.
   Do not run the Jetson profile as a substitute for flashing or validating the
   Jetson image.
3. In every new terminal after provisioning, activate the current shell and
   inspect readiness:

   ```bash
   source scripts/activate.sh
   ./scripts/bootstrap_dev.sh --check
   ```

4. From this repository, run `scripts/build.sh` and `scripts/test.sh` after
   code or configuration changes. They build the repository's packages into
   the enclosing `dev_ws/build`, `dev_ws/install`, and `dev_ws/log`
   directories.
5. For Phase 1 model inspection only, source the workspace and run
   `ros2 launch lb_sim mock_robot.launch.py use_rviz:=true` on a development
   host. This starts only upstream mock hardware and has no `/cmd_vel` path.
6. For the Phase 2 software bench, run
   `ros2 launch lb_launch bench_test.launch.py`. It uses only an in-memory
   mock and leaves mock output disabled by default.
7. `enable_motors:=true` is permitted only with the default mock transport and
   means in-memory mock motion. Do not set `use_mock_hardware:=false` expecting
   a real test: the Phase 2 skeleton intentionally fails closed.
8. Do not connect or enable physical propulsion through this software. Complete
   `DRIVE-02`, `DRIVE-03`, `SAFE-01`, and the ordered hardware tests before any
   real motor bench work.

The Orin Nano's firmware and JetPack image, the ZED SDK installation, power
adapter verification, and all hardware calibration are physical-target tasks.
They are not performed by bootstrap, which also does not edit a shell profile,
launch hardware, or validate connected devices. A successful provisioning run
can prepare mock and webcam-AprilTag development dependencies, but it is not
physical-robot or ZED readiness. RViz remains opt-in and should run offboard
rather than on the competition Jetson. See [bootstrap and activation](bootstrap.md)
for the full boundary.

## Phase 4.1 webcam AprilTag bench

This camera bench is non-actuating. It requires a connected webcam, the
user-confirmed 640 x 480 calibration, and a visible `tag36h11` ID 0 tag with a
0.250 m detector-corner edge. Before starting it, resolve the selected stable
camera path and do not substitute an arbitrary `/dev/videoN` enumeration.

```bash
source scripts/activate.sh
ros2 launch lb_sensors webcam_apriltag.launch.py
```

The launch uses `v4l2-ctl` from `v4l-utils` to set and verify the 15 Hz V4L2
source rate before it starts `v4l2_camera`, `image_proc` rectification, and
`apriltag_ros`. If the device, utility, requested 10–15 Hz rate, or read-back
verification fails, it aborts rather than silently running a different source
rate. In another sourced terminal, inspect the source-scoped output and its
newly stamped dynamic observation TF while the tag is visible:

```bash
ros2 topic echo /sensors/webcam/tag_detections --once
ros2 run tf2_ros tf2_echo webcam_optical_frame webcam_observation_tag_0
scripts/record_bag.sh --profile webcam_apriltag
```

For recording from a fresh terminal after the launch is running, use
`scripts/start_bag.sh` from the repository root. It activates the built
workspace and starts the webcam AprilTag profile by default. On the current
Jazzy workstation, run `scripts/start_bag.sh --ros-distro jazzy`; this explicitly
opts into non-target development use. Select another group with
`--profile hardware` (or another recorder profile), and preview with
`--dry-run`. Preserve the launch terminal's `ROS_DOMAIN_ID` and any custom
middleware/discovery environment settings. The script does not start nodes or
install software; unavailable selected topics can appear later and are not
evidence of successful capture until messages arrive. Press Ctrl+C to finish
recording. The printed output directory is under the repository's `bags/`;
inspect it with `ros2 bag info <directory>` and preserve the entire directory.

Add `--name tag_test_01` to save to `bags/tag_test_01` instead of a timestamped
directory, for example `scripts/start_bag.sh --ros-distro jazzy --name tag_test_01`.
Names accept letters, digits, underscores, and hyphens; existing outputs are
rejected. The same flag is available on `scripts/record_bag.sh`.

Do not interpret the observation as a robot or global pose. There is no
physical `base_link -> webcam_optical_frame` extrinsic, tag map, localizer,
EKF, `map`/`odom` transform, or motion path in this phase. Record the device
identity, calibration/detector configuration, V4L2 rate preflight, output, and
bag evidence before changing the camera, tag, port, resolution, or detector
settings.

A TF client can retain the last dynamic observation after the tag leaves view.
Record and enforce transform timestamp freshness in a future localizer; a
cached transform is not a current detection.

`ros2 launch lb_launch bringup.launch.py enable_apriltags:=true` provides the
same sensor-only bench entry point. It deliberately suppresses the Phase 1/2
mock bench and its synthetic camera transform; do not use it to connect the
real webcam observation to mock robot geometry.

## Phase 4.2 known-tag localizer

`tag_localizer` is a non-actuating measurement node, but it is intentionally
not part of normal bringup yet. Its launch requires explicit files:

```bash
ros2 launch lb_localization tag_localizer.launch.py \
  tag_map_file:=/absolute/path/to/surveyed_tag_map.yaml \
  localizer_config_file:=/absolute/path/to/surveyed_localizer.yaml
```

Do not substitute the installed templates: they contain `TBD` values and
unverified tag poses/extrinsics, so the launch/node will fail closed. Do not
reuse the Phase 1 mock webcam transform. A future physical run requires a
surveyed `map` tag map, measured `base_link -> camera_optical_frame` transform,
unique camera observation prefixes, and recorded gate/covariance provenance.

The package's synthetic test fixtures are for automated tests only. They may
be opted into only with `allow_synthetic_test_data:=true`; do not place that
argument in robot bringup or treat its pose output as a field pose. The node
publishes `/localization/apriltag_pose` and `/diagnostics` but never publishes
`map -> odom`. Until Phase 3 local odometry and a validated global EKF exist,
there is no live global-localization or navigation path.

## Static arena map

Install the dependencies declared by `lb_navigation` through the supported
bootstrap/rosdep workflow. On the current opt-in Jazzy development host, the
equivalent packages are `ros-jazzy-nav2-map-server` and
`ros-jazzy-nav2-lifecycle-manager`; this does not validate the target Humble
runtime.

After building and sourcing the workspace, launch the non-actuating map profile:

```bash
ros2 launch lb_navigation map.launch.py
```

The default launch uses the installed `maps/arena.yaml`, autostarts the
`map_server` lifecycle node, and opens the installed top-down RViz profile.
For a headless check, set `use_rviz:=false`. In another sourced terminal:

```bash
ros2 lifecycle get /map_server
ros2 topic echo /map --once --field header
ros2 topic echo /map --once --field info
ros2 topic info /map --verbose
```

Expected state is `active`; the grid frame is `map`, resolution is 0.05 m,
width/height are 139/102 cells, and origin position is `(-0.05, -0.05)`. The
outer occupied cell is outside the 6.85 m x 5.00 m free interior. This launch
publishes no TF or robot pose. Do not use RViz alignment alone as a physical
arena survey, and do not add the center column until its pose and dimensions
are measured.

The map launch also starts the visualization-only arena scene by default. The
light-yellow outline uses the Figure 9 nominal `6.88 x 5.0 m` dimensions; a
blue line marks `x=2.5 m`, a red rectangle marks the target berm, and a dashed
green rectangle marks the provisional starting zone. Orange AprilTag markers
show configured assumed/surveyed tag planes and face normals. The central
column, rocks, and craters are omitted. Disable this display publisher with
`use_arena_scene:=false`; doing so does not change `/map`.

## Provisional robot-pose replay in the arena

This profile is a non-actuating development aid for `bags/test1_AT1`. It treats
the hand-moved recorded webcam as though it were rigidly mounted to the robot,
so its arrow is useful for checking transform direction, motion shape, loss,
and reacquisition—not localization accuracy.

Install the map-server dependencies listed above, build, and source the
workspace. From the repository root, run:

```bash
ros2 launch lb_sim arena_pose_replay.launch.py \
  bag_path:=$PWD/bags/test1_AT1 \
  playback_rate:=0.25
```

The default RViz profile shows the arena map, durable arena scene and assumed
tag plane, recorded image, raw camera RGB axes, cyan planar `base_link` arrow,
and current
contiguous planar path. It also shows a translucent blue 0.90 m x 0.60 m x
0.25 m synthetic chassis, yellow chassis-only footprint, pink rear webcam,
and green forward ZED. The two cameras are provisionally colocated in X/Y with
`base_link`, 0.40 m above it; the webcam points toward robot `-X` and the ZED
points toward robot `+X`. Their colored arrows show those view directions.
Because the bodies overlap, use the arrows and colors to distinguish them.
Tag 1 is provisionally centered
at `(0.0, 1.0, 0.75)` m and faces arena `+X`. Both transforms are estimates.

The chassis dimensions are reused from the synthetic mock profile and are not
cart measurements. The footprint excludes the scoop and is not approved as a
Nav2 collision footprint. When the tag leaves view for more than 0.5 s, all
dynamic robot/camera markers are deleted, the path is cleared, and no dead
reckoning occurs. Reacquisition
starts a new path segment. The bag's `--loop` clock rewind also clears state.
Inspect interfaces in another sourced terminal if needed:

```bash
ros2 topic echo /visualization/replay/webcam_pose_raw --once
ros2 topic echo /visualization/replay/base_pose_planar --once
ros2 topic echo /diagnostics
```

For a headless pose-only check when Nav2 map-server packages are unavailable,
add `use_rviz:=false use_map_server:=false`. This omits `/map` but still replays
the pose adapter. Do not feed these visualization topics into Nav2, an EKF, or
robot control. Replace the tag pose and camera extrinsic only with documented
survey/calibration values; production localization remains the C++ localizer
plus the future local/global EKFs.
