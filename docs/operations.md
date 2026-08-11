# Operations

## Profiles

- **Development host:** may run static checks and mock/simulation work. It does
  not validate the Jetson runtime lock.
- **Jetson bench:** requires the locked JetPack/L4T, Humble, ZED SDK, and
  wrapper versions to be verified on the physical target before sensor work.
- **Competition:** headless runtime; motors disabled by default and RViz run
  offboard where possible.

## Safe setup

1. Review `docker/versions.env` and `docs/resources/software_platform_lock.md`.
2. Run `scripts/bootstrap_dev.sh --check`; it performs no installation.
3. From this repository, run `scripts/build.sh` and `scripts/test.sh`. They
   build the repository's packages into the enclosing `dev_ws/build`,
   `dev_ws/install`, and `dev_ws/log` directories.
4. For Phase 1 model inspection only, source the workspace and run
   `ros2 launch lb_sim mock_robot.launch.py use_rviz:=true` on a development
   host. This starts only upstream mock hardware and has no `/cmd_vel` path.
5. For the Phase 2 software bench, run
   `ros2 launch lb_launch bench_test.launch.py`. It uses only an in-memory
   mock and leaves mock output disabled by default.
6. `enable_motors:=true` is permitted only with the default mock transport and
   means in-memory mock motion. Do not set `use_mock_hardware:=false` expecting
   a real test: the Phase 2 skeleton intentionally fails closed.
7. Do not connect or enable physical propulsion through this software. Complete
   `DRIVE-02`, `DRIVE-03`, `SAFE-01`, and the ordered hardware tests before any
   real motor bench work.

The Orin Nano's firmware and JetPack image, the ZED SDK installation, power
adapter verification, and all hardware calibration are physical-target tasks.
They are not performed by the bootstrap script. RViz remains opt-in and should
run offboard rather than on the competition Jetson.
