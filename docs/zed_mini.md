# ZED Mini on the Jetson and mission control laptop

`lb_sensors/launch/zed_mini.launch.py` starts the Stereolabs ROS 2 wrapper for
`camera_model:=zedm`. The USB camera and ZED SDK stay on the Jetson. The wrapper
computes depth there and advertises ROS 2 image, depth, IMU, and point cloud
topics. Mission control can view them from the laptop over the router's LAN.
The laptop needs ROS 2 Jazzy, a matching ROS domain, RViz 2, and the
`ros-jazzy-compressed-image-transport` plugin for compressed video, but no ZED
SDK just to view standard topics in RViz 2.

The supplied `zed_mini.yaml` grabs HD720 at 30 Hz for local SDK processing,
publishes half-size images and depth at up to 10 Hz, and caps the optional point
cloud at 5 Hz. Image and depth data are published when subscribed. It leaves
SDK object detection, positional tracking, and mapping off. The separate,
opt-in terrain node below detects obstacle geometry; it does not command a
stop. The wrapper publishes the camera's own URDF/optical frames, but its
odom/map TF is disabled until the physical mount and localization design are
measured. Do not also publish the mock robot's
synthetic ZED mount into the same TF graph.

## Jetson setup and manual start

Install and verify the locked JetPack, ZED SDK, ROS 2 Jazzy, and wrapper on the
Jetson. Install the wrapper's dependencies using its README/`rosdep` instructions,
then build it in the same workspace as this repository. The repository's
`scripts/build.sh` builds only the `lb_*` packages, so build the separate
Stereolabs checkout first:

```bash
cd ~/dev_ws/src/jmu-lunabotics
./scripts/build_zed_wrapper.sh
./scripts/build.sh
```

The ZED build script runs one package and one compiler job at a time. The
previous `--parallel-workers 1` setting limited packages but could still let
CMake run one compiler job per CPU core. This build will take longer. If the
Jetson still reboots during a single-job build, inspect its power supply and
cooling; a script cannot prevent a hardware reset.

Check that `ros2 pkg prefix zed_wrapper` succeeds and the camera is visible to the ZED
SDK before enabling boot startup.

```bash
cd ~/dev_ws/src/jmu-lunabotics
source scripts/activate.sh
ros2 launch lb_launch bringup.launch.py enable_zed:=true
```

This public entry point runs the camera without the mock drive/robot model.
For a sensor-only start, use `ros2 launch lb_sensors zed_mini.launch.py`.
If several ZED cameras are connected, pass the actual serial number with
`zed_serial_number:=<serial>` to `lb_launch` or `serial_number:=<serial>` to
`lb_sensors`. `zed_params_file:=/absolute/path/to/override.yaml` accepts a
deployment-specific parameter file. With the default camera name, useful topics
are `/zed/zed_node/rgb/color/rect/image`, its `/compressed` transport, and
`/zed/zed_node/depth/depth_registered`. Inspect the live graph with:

```bash
ros2 topic list | rg '^/zed/'
ros2 topic hz /zed/zed_node/rgb/color/rect/image
```

## Start at Jetson boot

The provided systemd user service starts the sensor-only launch and restarts
it if the process exits. It expects this repository at
`~/dev_ws/src/jmu-lunabotics` and the built overlay at `~/dev_ws/install`.
After a successful manual camera run, on the Jetson:

```bash
systemctl --user link "$HOME/dev_ws/src/jmu-lunabotics/systemd/lunabot-zed.service"
systemctl --user enable --now lunabot-zed.service
sudo loginctl enable-linger "$USER"
systemctl --user status lunabot-zed.service
journalctl --user -u lunabot-zed.service -f
```

For stable selection with multiple cameras, run
`systemctl --user edit lunabot-zed.service`, add the following override, then
run `systemctl --user daemon-reload` and
`systemctl --user restart lunabot-zed.service`:

```ini
[Service]
Environment=ZED_SERIAL_NUMBER=12345678
```

Run either this service or a separate boot service using
`lb_launch bringup.launch.py enable_zed:=true`, not both, because one USB camera
cannot be opened by two wrapper processes.

## Terrain hazard detector

The opt-in `terrain_hazard_node` runs on the Jetson. It samples the registered
depth image and its `CameraInfo`, transforms samples to `base_link` at the image
timestamp, fits nearby ground, and publishes two stamped `PointCloud2` topics:
`/perception/depth_obstacles` for measured raised rock cells and
`/perception/terrain_hazards` for measured drops, steep terrain, and corroborated
unsafe rims. Its synthetic hazard points are placed just above the estimated
ground surface so a future costmap can mark them. Missing depth is counted in
`/diagnostics`; an isolated invalid pixel is not a crater. It publishes empty
clouds only after a fresh frame with matching calibration, measured TF, enough
depth coverage, and a valid ground estimate. Stale, sparse, or untransformable
frames produce diagnostics without new clouds. A later costmap consumer must
handle hazard clearing and expiry explicitly.

There are no measured mount or robot clearance settings in this repository.
Copy `lb_sensors/config/terrain_hazard.template.yaml` to a robot-specific file
and provide a recorded `mount_calibration_id`, measured unsafe rock height,
minimum drop depth, and maximum traversable slope. Publish the surveyed
`base_link` to camera mount transform from the physical robot description;
the wrapper supplies its own camera optical frames. The synthetic Phase 1 mount
must never be used. The node refuses the template's unset values.

After physical calibration, start a manual sensor-only run with:

```bash
ros2 launch lb_launch bringup.launch.py enable_zed:=true \
  enable_depth_perception:=true \
  terrain_params_file:=/absolute/path/to/measured_terrain_hazard.yaml
```

To enable the same detector in the boot service, add these two lines to its
systemd user override, then reload and restart it:

```ini
[Service]
Environment=ENABLE_DEPTH_PERCEPTION=true
Environment=TERRAIN_PARAMS_FILE=/absolute/path/to/measured_terrain_hazard.yaml
```

Run `systemctl --user daemon-reload` and
`systemctl --user restart lunabot-zed.service`. Check `/diagnostics`, both
`/perception/*` topics, camera TF at image time, CPU/memory, and frame latency.
The detector exiting also stops this launch so the boot service can restart
it; an invalid template cannot leave a healthy-looking camera-only service
when depth perception was requested.
Replay a ZED recording and validate rocks, slopes, and crater/drop-off edges
on the Jetson before using these topics for navigation. The provisional
processing-latency target is 150 ms. This phase does not alter a static map,
configure Nav2, or turn an empty depth frame into a clear path.

For replay, record the registered depth image, matching `CameraInfo`, and the
surveyed `/tf` and `/tf_static` chain. Start only the detector with the measured
parameter file and simulated time, then play that bag with a clock:

```bash
ros2 run lb_sensors terrain_hazard_node --ros-args \
  --params-file /absolute/path/to/measured_terrain_hazard.yaml \
  -p use_sim_time:=true
# In another terminal:
ros2 bag play /absolute/path/to/zed_bag --clock
```

Repeat with the camera TF withheld and with stale depth timestamps: the node
must report errors on `/diagnostics` and publish no new obstacle clouds.

## View on the laptop

Connect the laptop Ethernet port and Jetson Wi-Fi to the same router LAN. The
router must bridge wired and wireless clients, have client isolation off, and
pass ROS 2 DDS discovery multicast and UDP data. Both hosts must use the same
`ROS_DOMAIN_ID` (the service uses `0`), and ROS discovery must not be limited to
localhost. NAT or guest Wi-Fi may prevent discovery. Check the laptop can see
`/zed/zed_node/rgb/color/rect/image/compressed` with `ros2 topic list` before
configuring RViz 2.

In RViz 2, add an **Image** display, set its topic to
`/zed/zed_node/rgb/color/rect/image`, and select **compressed** as the transport.
The compressed stream is much smaller over Wi-Fi. Use a point cloud display
only when needed; raw point clouds can be expensive on this link. RViz is a
viewer, while the ZED SDK remains the camera and depth engine on the Jetson.
If discovery works but video stutters, lower `general.pub_frame_rate`, raise
`general.pub_downscale_factor`, or use a compressed transport; keep
`general.grab_resolution` independent for local depth quality.

See [Stereolabs ROS 2 integration](https://docs.stereolabs.com/docs/integrations/ros-2)
and [DDS/network tuning](https://docs.stereolabs.com/docs/integrations/ros-2/dds-and-network-tuning/).
