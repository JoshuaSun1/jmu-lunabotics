# Software Platform Lock

**Status:** reported robot image; physical software and sensor validation pending

**Updated:** 2026-10-13

**Target compute:** NVIDIA Jetson Orin Nano Developer Kit / Super Developer Kit, 8 GB

## Robot software baseline

| Layer | Version / value | Basis |
|---|---|---|
| Jetson hardware | Jetson Orin Nano, 8 GB | Project hardware inventory. |
| JetPack | 7.2.1 | Reported installed robot release; NVIDIA lists Orin-family support. |
| Jetson Linux (L4T) | 39.2.1 | NVIDIA's JetPack 7.2.1 BSP. |
| Root filesystem | Ubuntu 24.04 | JetPack 7.2.1 L4T Ubuntu base. |
| Kernel | 6.8 | JetPack 7.2.1 release information. |
| CUDA | 13.2.2 | NVIDIA's JetPack 7.2.1 package version; verify the installed minor version on the robot. |
| ROS 2 | Jazzy Jalisco | Native Ubuntu 24.04 pairing with arm64 binary packages. |
| ZED SDK | 5.5.0 | Reported 5.5 installation; Stereolabs publishes a JetPack 7.2 / CUDA 13.2 build. Verify installed patch version. |
| ZED ROS 2 wrapper | `v5.5.0` Git tag | Matching wrapper release with Jazzy support. Record the resolved commit if added to the workspace. |
| Container architecture | `linux/arm64` | Required on the Jetson. |

The ZED SDK 5.5 release page labels the JetPack 7.2 build as L4T 39.2. Confirm
that the installed SDK package works with the robot's 39.2.1 patch release.
`docker/versions.env` describes a software baseline, not proof that the CUDA,
SDK, or wrapper patches were installed and tested on this robot.

## Sensor integration

- **Depth/IMU camera:** Stereolabs ZED Mini, connected to a dedicated USB 3 Type-A port.
- Use the ZED SDK and wrapper for camera/IMU data. Preserve the TF authorities in the autonomy design.
- The repository does not yet launch the ZED wrapper. Its commit, camera configuration, and sensor performance remain target validation work.

## Validation on the Jetson

1. Capture `cat /etc/nv_tegra_release`, `dpkg-query -W nvidia-jetpack`, `nvcc --version`, the ZED SDK version, and installed ROS package versions. The bootstrap `--profile jetson` check verifies the OS, architecture, L4T, and model only.
2. Install or build the `v5.5.0` wrapper against the installed ZED SDK and Jazzy, and record its commit SHA. Do not reuse a previous Humble workspace overlay; rebuild the workspace under Jazzy.
3. Verify ZED Mini RGB, depth, IMU timestamps, TF ownership, and CPU/GPU/memory use at the intended operating rate.
4. Record exact installed package versions and any validated container image digest. The generic Ubuntu build container does not contain JetPack or the ZED SDK.
5. Test the complete stack on the physical robot before treating it as competition-ready.

## Other hardware/software dependencies still TBD

- 2D LiDAR model, driver, serial settings, and mounting transform.
- Drive geometry, wheel radius, effective wheel separation, gear ratio, encoder resolution, CAN IDs, and SPARK MAX configuration.
- Motor-controller CAN protocol and the command/feedback interface behind the ODrive USB-to-CAN adapter.
- Jetson robot-power adapter: verify the regulated 19 V barrel-jack input, connector dimensions, polarity, and current capacity.

## Sources

- [NVIDIA JetPack 7.2.1 release information](https://developer.nvidia.com/embedded/jetpack/downloads): Orin support, L4T 39.2.1, Ubuntu 24.04, kernel 6.8, and CUDA 13.2.2.
- [NVIDIA Orin Nano BSP setup](https://docs.nvidia.com/jetson/orin-nano-devkit/user-guide/setup_bsp.html): JetPack 7.2.1 installation and firmware requirements.
- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html): Noble amd64 and arm64 binary packages.
- [Stereolabs ZED SDK 5.5 release](https://www.stereolabs.com/en-gb/developers/release/5.5): JetPack 7.2 / CUDA 13.2 installer for Orin and Thor.
- [Stereolabs ZED ROS 2 wrapper releases](https://github.com/stereolabs/zed-ros2-wrapper/releases/tag/v5.5.0): `v5.5.0`; [wrapper support matrix](https://github.com/stereolabs/zed-ros2-wrapper) lists Jazzy.
