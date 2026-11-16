# N5–N8 mock navigation

This is an explicit development profile using real Jazzy Nav2 servers and the
repository's in-memory four-wheel drive transport. Physical use is prohibited.
The normal robot bringup is unchanged. The profile starts nothing unless
`enable_nav2`, `allow_mock_navigation`, and `allow_test_only_tf` are all true;
it rejects real hardware and physical motor enable requests.

## Start and operate

Build after installing the declared dependencies. After new dependencies are
installed, force CMake to rediscover previously unavailable optional targets.
Run this block from the workspace root (`~/dev_ws` in the canonical layout):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-force-configure
source install/setup.bash
ros2 launch lb_sim nav2_mock.launch.py enable_nav2:=true \
  allow_mock_navigation:=true allow_test_only_tf:=true use_rviz:=true
```

Run these commands from another sourced terminal. Establish a current
`/localization/fused_pose` and test TF before lifecycle startup:

```bash
ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes '{command: 0}'
ros2 topic echo /navigation/ready --once
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.5}, orientation: {w: 1.0}}}}' --feedback
```

`/navigation/ready` must be true. The public action rejects invalid goals,
custom behavior-tree paths, concurrent goals and unavailable prerequisites.
The RViz profile shows map, costmaps, footprint, fused pose, TF and global plan;
its Nav2 panel/goal tool uses the public guarded action. Action feedback/results,
diagnostics and commands can also be inspected with ROS CLI. RViz visual review
requires a graphical host; automated acceptance is headless.

## Data and command ownership

```text
/cmd_vel -> mock diff_drive_controller -> in-memory DriveTransport
                         |
                         v
                    /odom/wheel
                         |
              navigation_mock_world
                 |                |
                 v                v
synthetic AprilTag pose     /odometry/local, /sim/scan, /sim/hazards
                 |
        fused_pose_passthrough -> /localization/fused_pose + /localization/valid
                 |
         opted-in N3 development TF bridge
             map -> odom -> base_link

planner + controller -> velocity smoother -> existing localization gate
 /plan       /cmd_vel_nav_controller          /cmd_vel_nav_raw -> /cmd_vel_nav
                                                            |
                                                    navigation_supervisor
                                                            |
                                                         /cmd_vel
```

The synthetic world supplies measurement-stamped pose from wheel odometry with
an initial offset of (1.5, 1.5) m. It is synthetic truth, not camera detection,
EKF fusion or validated localization. Its freshness check stops all outputs if
wheel samples are missing, future-dated or older than 0.25 s. The existing N3
bridge remains the sole test publisher of map/odometry edges; the drive
controller's odometry TF stays disabled. The local-odometry topic is also a
mock fixture. No EKF is started.

`bt_navigator` serves `/navigation/internal/navigate_to_pose`; the supervisor
owns `/navigate_to_pose`. The supervisor admits a goal only with current
localization, both TF edges, wheel-derived local odometry, both synthetic sensor
streams, active Nav2 lifecycle states, and an available internal navigator.
Only an active admitted goal can authorize final commands. Stale/invalid command
stamps, invalid frames, lateral/vertical motion, nonfinite values or excessive
speeds produce zero. Loss of prerequisites aborts the public action, cancels the
internal action and publishes zero; reacquisition cannot resume that goal.
Pending internal cancellations lock new goals until acknowledged. If the
navigator dies without acknowledgement, restart the development profile.

This is single-navigation-source command arbitration. Teleoperation priority,
a production safety supervisor, E-stop integration and physical motor enable
are separate future work. The final software publisher is the supervisor;
controller/transport timeouts provide a separate mock-drive stop boundary.
The Phase 2 child launch's `enable_motors=true` selects only in-memory output
because its transport is hardcoded to mock; it is not physical enablement.

## Plugin and parameter rationale

NavFn provides a small, established 2D planner for the known arena. Regulated
Pure Pursuit provides bounded differential-drive path following and footprint
collision prediction. No path smoother is added without evidence of need.
PoseProgressChecker counts both translation and rotation: an XY-only checker
incorrectly declared deliberate heading alignment stalled in the first mock
run. SimpleGoalChecker handles final position/yaw tolerances. Wait and costmap
clearing are the only recovery behaviors; the BT permits two retries and
replans at 1 Hz. Backup, spin recovery, MPPI, waypoint missions and autonomous
mechanisms are not enabled. Heading alignment by the controller is distinct
from an unconstrained spin recovery.

All new numbers are synthetic test policy, not measured robot/device limits:

| Setting | Mock value / provenance |
|---|---|
| Polygon, padding, speed/acceleration/jerk envelope | Existing `robot_limits_development.yaml`; team estimate and Phase 2 synthetic limits |
| Desired path speed / heading rate | 0.25 m/s, 0.5 rad/s; conservative mock starting values below the existing 0.5 m/s and 1 rad/s envelope |
| Progress / final tolerances | 0.05 m or 0.1 rad within 3 s; final 0.1 m / 0.15 rad; mock test policy |
| Pose/TF/sensor maximum age | 0.50 s, existing development policy |
| Command timeout / gate period | 0.25 / 0.05 s; mock watchdog policy |
| Lifecycle poll / response age / bond timeout | 0.25 / 1.0 / 4.0 s; development liveness policy |
| Goal deadline | 120 s; mock failure bound |
| Costmap rates / initial TF timeout | Global update/publish 5/2 Hz, local 10/5 Hz; initial TF timeout 10 s accommodates startup |
| Grid / window / inflation | N4 values: 0.05 m/cell, 4 m rolling window, 0.45 m inflation, factor 3.0; untuned |
| Synthetic scan | 20 Hz, 721 samples including both angular endpoints, full circle, 0.05–8 m; deterministic generator settings |
| Synthetic hazards | 9 x 9 points, 0.05 m spacing, z=0.1 m; obstacle representation fixture |
| Synthetic source stale indication | 0.2 s expected update interval; independent supervisor stops at 0.5 s |
| Synthetic pose covariance | 0.001 for x/y/yaw; fixture value, not an accuracy claim |

Remaining lookahead, collision horizon, cost and timing parameters are explicitly
in `nav2_mock_development.yaml`; all belong to the same untuned mock profile.
Parameter changes require rerunning the acceptance suite. NavFn's grid paths
are not a proof of physical rectangular-footprint clearance; RPP's collision
checks and representative-path testing complement the planner.

## Synthetic obstacles and fault injection

```bash
ros2 param set /navigation_mock_world scenario block
# Other scenarios: clear, wall, hazard
ros2 service call /navigation_mock_world/scenario/localization_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /navigation_mock_world/scenario/scans_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /navigation_mock_world/scenario/hazards_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /navigation_mock_world/scenario/freeze_pose std_srvs/srv/SetBool '{data: true}'
```

Restore streams by setting their service values true; unfreeze with false.
Reissue a new goal after readiness returns. `freeze_pose` simulates a stalled
robot while retaining fresh synthetic timestamps to exercise progress failure.

`block` is a rectangle at x=2.3–2.7, y=1.1–1.9 m; `wall` spans x=2.4–2.6,
y=0–5 m. `hazard` marks a synthetic patch starting at (2.3, 1.3) m.
`clear` removes synthetic scan obstacles; raytracing clears them. Hazard points
persist in their layer until explicit costmap clearing because absence of
terrain points is not proof a drop-off disappeared. Set `scenario=clear` before
clearing if the patch should disappear. Recovery clearing cannot permanently
remove a still-published hazard. All geometry is synthetic arena-frame data.

The `reset_pose` SetBool service teleports the synthetic world to (1.5, 1.5) m,
retaining wheel yaw. Use only while idle; it does not reset physical hardware.
The acceptance test uses explicit resets between independent fault scenarios.

## Automated acceptance and evidence

From the repository root, with ROS/workspace sourced and local DDS allowed:

```bash
LB_MOCK_EVIDENCE_DIR=/tmp/lb_mock_acceptance \
  python3 -m pytest -q lb_sim/test/test_navigation_mock_runtime.py -s
# Add a short repeat-navigation run for resource evidence:
LB_MOCK_SOAK_SECONDS=60 LB_MOCK_EVIDENCE_DIR=/tmp/lb_mock_soak \
  python3 -m pytest -q lb_sim/test/test_navigation_mock_runtime.py -s
```

The test starts and stops its own profile and deliberately faults only its own
launch process group. It covers initial goal rejection, two completed goals,
bounded final commands, cancellation, localization/scan/hazard loss, no implicit
resume, live replanning around a new obstacle, scan clearing, hazard marking,
unreachable goals, no progress, recovery exhaustion, controller stall and
planner-process loss. It saves `launch.log` and `results.json` with action
results, latency, command age, and process-group CPU/RSS samples.

CPU is expressed as summed core-percent (100% means one core). RSS sums resident
pages across processes and double-counts shared libraries; it is not unique or
Jetson GPU memory. Default CI runs omit the optional repeat-navigation period.
The repeat-navigation period requires at least two completed goals; freshness
failures are recorded as safety aborts with explicit stop/readiness/new-goal
checks, and may not outnumber successful traversals. It does not widen freshness
limits to mask host scheduling delays. Unexpected action errors fail the run.

A short x86 smoke/soak is not field endurance, target timing, braking-distance,
physical collision clearance or hardware safety evidence.

## Later hardware integration

Replace the synthetic world with calibrated wheel odometry/IMU, surveyed
AprilTags and measured sensor transforms. Replace the N3 bridge with sole-owner
local/global EKFs and keep the fused-pose application interface. Replace the
`/sim` costmap observations with validated device outputs and measured stale,
range/height and negative-obstacle semantics. Replace development geometry and
limits, implement the MCU protocol and hardware watchdog/E-stop/enable/reset
path, then establish production command arbitration and goal supervision.
These are separately gated tasks, not a flag that enables this mock profile on
the robot. Keep the mock profile and acceptance suite for regression testing.
