# Performance budget

Target compute is an actively cooled Jetson Orin Nano 8 GB. Phase 0 sets no
measured performance claim.

The planned initial runtime profile is headless, 720p-or-lower ZED depth/RGB,
15 FPS depth, 10–15 FPS AprilTag processing, no dense point cloud by default,
and no ZED positional tracking or object detection. Future profiling must
record `tegrastats`, node CPU/memory, topic rates/bandwidth, callback latency,
dropped frames, and costmap timing.

Acceptance targets include no OOM or sustained thermal throttling during a
30-minute representative run, at least 10–15% shared-memory headroom, and
depth-hazard latency below 150 ms. These are engineering targets pending
measurement, not current guarantees.
