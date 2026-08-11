# TBD register

This register prevents assumptions from becoming accidental implementation
decisions. A value may be closed only with evidence, units where applicable,
and a recorded validation method.

| ID | Area | Required decision/value | Needed by | Closure criterion |
|---|---|---|---|---|
| PLAT-01 | Runtime | Physical Orin revision, firmware, installed packages, image digest | Phase 1 | Target boot and version capture |
| PLAT-02 | Development | x86 build/CI strategy and arm64 validation runner | Phase 0 | Documented, reproducible workflow |
| POWER-01 | Power | Orin 19 V adapter connector, polarity, current capacity, fuse/converter | Before robot power-up | Electrical review and bench test |
| DRIVE-02 | Drive | Wheel radius, separation, gear ratio, encoder source/resolution, signs | Phase 1–2 | Measured calibration record |
| DRIVE-03 | Motor comms | MCU role, CAN topology/bitrate/IDs, SPARK MAX configuration, heartbeat/fault protocol | Before real transport | Approved protocol specification |
| SAFE-01 | Safety | E-stop path, enable/reset policy, battery/current/tilt/staleness thresholds | Before real output / Phase 9 | Electrical and safety review |
| GEOM-01 | Robot | Frame origin, footprint(s), clearance, speed/acceleration/jerk limits | Phase 1 onward | Measured and reviewed values |
| ZED-01 | ZED Mini | Unit/firmware, USB port, mount, transform, calibration, timestamp/frame authority | Phase 1 / 3 | Bench validation record |
| LIDAR-01 | LiDAR | Model, driver, connection, range/filter/mount settings | Phase 1 / 6 | Selected hardware and driver test |
| TAG-01 | AprilTags | Camera choice, family, IDs, sizes, placement, world poses, uncertainty | Phase 4 | Surveyed tag map |
| MAP-01 | Arena | Map, resolution, origin, occupancy thresholds, keepouts | Phase 4–5 | Calibrated map artifact |
| LOC-01 | Localization | EKF covariances, IMU axes/yaw, tag quality gates, validity thresholds | Phase 3–4 | Recorded-data validation |
| PER-01 | Perception | ROI, terrain thresholds, negative-obstacle strategy, costmap parameters | Phase 6–7 | Arena test evidence |
| MISS-01 | Mission | Excavation/deposition interfaces, limits, feedback, recovery behavior | Phase 8 | Interface contract |
| OPS-01 | Operations | Teleop device, network/time sync, rosbag storage/retention, enable procedure | Phase 5 onward | Operations review |

## BOM observations

The BOM identifies an Orin Nano 8 GB, ZED Mini, six NEO 2.0 motors, six SPARK
MAX controllers, and an ODrive USB-to-CAN adapter. It does **not** select a
2D LiDAR or microcontroller. The listed Jetson-Nano-labelled power adapter is
not validated for the Orin until `POWER-01` closes.

On 2026-08-10, the team confirmed that the chassis will have four driven
wheels and a large scooping bucket. The other ordered motors are intended for
the digging arm rather than additional drive wheels. This closes the six-wheel
interpretation of the BOM. The 2026-08-10 design handoff and team confirmation
also establish skid/tank steering. They do not establish dimensions, actuator
interfaces, or calibrated drive values.

## Resolved design decisions

| ID | Decision | Evidence | Remaining boundary |
|---|---|---|---|
| DRIVE-01 | Four-wheel skid-steer/tank topology; no steering linkage or steering joints. | Team confirmation and [`Current Lunabot Robot Design Codex Handoff`](resources/Current_Lunabot_Robot_Design_Codex_Handoff.md), 2026-08-10. | `DRIVE-02`, `DRIVE-03`, and `GEOM-01` remain open; Phase 2 must still validate controller configuration and wheel groups. |
