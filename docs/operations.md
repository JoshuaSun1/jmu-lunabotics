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
3. Build and test only within this repository using `scripts/build.sh` and
   `scripts/test.sh`.
4. Do not connect or enable propulsion through software during Phase 0.

The Orin Nano's firmware and JetPack image, the ZED SDK installation, power
adapter verification, and all hardware calibration are physical-target tasks.
They are not performed by the bootstrap script.
