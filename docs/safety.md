# Safety

Phase 0 establishes no active safety controller. The design constraint is that
the physical emergency stop must remove propulsion independently of the Jetson,
ROS, Wi-Fi, or mission software.

Phase 2 adds only software fail-closed behavior inside the drive hardware
boundary: the mock begins disabled, the real transport skeleton cannot connect
or enable output, and failed state reads/writes cause zero commands plus
`stop()` and `set_enabled(false)` requests before a ROS-control error is
returned. This does not implement physical propulsion control, an emergency
stop, a microcontroller heartbeat, fault latching, or a safe-reset path.

Future safety work will monitor command, wheel-odometry, IMU, localization,
LiDAR/depth (during autonomy), battery, controller faults, pitch/roll, and
Jetson-to-microcontroller heartbeat. Severe faults must latch until an explicit
safe reset. Thresholds, hardware wiring, and reset behavior remain `TBD`.
