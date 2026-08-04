# Safety

Phase 0 establishes no active safety controller. The design constraint is that
the physical emergency stop must remove propulsion independently of the Jetson,
ROS, Wi-Fi, or mission software.

Future safety work will monitor command, wheel-odometry, IMU, localization,
LiDAR/depth (during autonomy), battery, controller faults, pitch/roll, and
Jetson-to-microcontroller heartbeat. Severe faults must latch until an explicit
safe reset. Thresholds, hardware wiring, and reset behavior remain `TBD`.
