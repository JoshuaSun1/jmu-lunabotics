# Software Platform Lock

**Status:** proposed development and robot-runtime baseline
**Last researched:** 2026-08-04
**Target compute:** NVIDIA Jetson Orin Nano Developer Kit / Super Developer Kit, 8 GB

## Locked robot-runtime baseline

| Layer | Version / value | Rationale |
|---|---|---|
| Jetson hardware | Jetson Orin Nano, 8 GB | Matches the project target and current BOM. |
| JetPack | 6.2.2 | Latest production JetPack 6 release listed for the Orin Nano family at the research date. |
| Jetson Linux (L4T) | 36.5.0 | The BSP supplied by JetPack 6.2.2. |
| Root filesystem | Ubuntu 22.04 | The JetPack 6.2.2 root filesystem base. |
| Kernel | 5.15 | Supplied by Jetson Linux 36.5. |
| CUDA | 12.6 | Supplied by JetPack 6.2.2. |
| ROS 2 | Humble Hawksbill | Native Ubuntu 22.04 LTS pairing; use this for robot runtime rather than Jazzy. |
| ZED SDK | 5.2.3 | Stereolabs publishes this release for JetPack 6.2.2 / L4T 36.5. |
| ZED ROS 2 wrapper | `v5.2.2` Git tag | Compatible with ZED SDK 5.2 and ROS 2 Humble. Pin the resolved commit SHA when vendored. |
| Container architecture | `linux/arm64` | Required on the Jetson. |

## Sensor integration resolved by the BOM

- **Depth/IMU camera:** Stereolabs ZED Mini.
- Connect the ZED Mini to a dedicated Jetson USB 3 Type-A port; it is a USB 3 UVC camera.
- Use the ZED SDK and wrapper only for camera/IMU data. The autonomy architecture remains responsible for localization fusion and must preserve the specified TF authorities.

## Required validation before declaring this production-ready

1. Update the Jetson developer-kit firmware before installing a JetPack 6 image if the kit still has factory firmware.
2. Flash a clean JetPack 6.2.2 image, install the exact ZED SDK above, and build the pinned wrapper on the physical Orin Nano.
3. Verify ZED Mini images, depth, IMU timestamps, and CPU/GPU/memory use at the intended operating rate.
4. Record exact installed Debian package versions and container image digest in `docker/versions.env` once the Phase 0 scaffold exists.
5. Test the complete stack on the physical robot before treating these versions as final competition locks.

## Important hardware/software dependencies still TBD

- 2D LiDAR model, driver, serial settings, and mounting transform.
- Drive geometry, wheel radius, effective wheel separation, gear ratio, encoder resolution, CAN IDs, and SPARK MAX configuration.
- Motor-controller CAN protocol and the command/feedback interface behind the ODrive USB-to-CAN adapter.
- Jetson robot-power adapter: it must provide a regulated 19 V barrel-jack input with verified connector dimensions, polarity, and current capacity. The BOM's adapter is labelled for a Jetson Nano, so it must be electrically checked before use.

## Sources

- NVIDIA, [JetPack 6.2.2](https://developer.nvidia.com/embedded/jetpack-sdk-622): L4T 36.5, Ubuntu 22.04 root filesystem, kernel 5.15, CUDA 12.6, and Orin-family support.
- NVIDIA, [Jetson Orin Nano Developer Kit getting-started guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit): 19 V barrel-jack input, USB 3 ports, and JetPack-6 firmware prerequisite.
- Stereolabs, [ZED SDK 5.2 release](https://www.stereolabs.com/en-lu/developers/release/5.2): ZED SDK 5.2.3 support for JetPack 6.2.2 / L4T 36.5.
- Stereolabs, [ZED ROS 2 wrapper](https://github.com/stereolabs/zed-ros2-wrapper): ROS 2 Humble support, ZED SDK 5.2 prerequisite, and `v5.2.2` release tag.
- Stereolabs, [ZED Mini product page](https://store.stereolabs.com/products/zed-mini): Jetson Orin support and USB 3 connection.
