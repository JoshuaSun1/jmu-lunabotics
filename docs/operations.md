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
