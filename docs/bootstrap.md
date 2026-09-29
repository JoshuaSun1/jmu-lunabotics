# Bootstrap and activation

The bootstrap workflow separates **one-time host provisioning** from
**per-terminal activation**. It supports the locked robot software baseline:
ROS 2 Humble on Ubuntu 22.04 (Jammy). It is not an image-flashing or
hardware-validation tool.

## Supported profiles

| Profile | Intended device | Use only when | Purpose |
|---|---|---|---|
| `development` | A Jammy development computer | The host is running Ubuntu 22.04 and will be used for ROS development, mock, or webcam-AprilTag work | Installs the supported ROS/development dependencies declared by this repository and prepares the workspace. |
| `jetson` | The selected Jetson Orin Nano | JetPack 6.2.2 / L4T 36.5 has already been manually installed and verified on the arm64 target | Installs the ROS workspace dependencies for the target after its platform preflight. |

An x86_64 Ubuntu 24.04/Jazzy machine is useful for structural development
checks only. It is not a substitute for the Jammy/Humble provisioning path or
for the arm64 Jetson runtime.

The Jetson profile checks Ubuntu 22.04, arm64, L4T 36.5, and an Orin Nano model
string. Confirm the physical 8 GB SKU separately: the device-tree model does
not encode memory capacity.

## One-time provisioning

For a normal developer setup, clone the repository into the canonical ROS
workspace layout before provisioning. Substitute the team remote for the
placeholder below:

```bash
mkdir -p ~/dev_ws/src
git clone <team-repository-url> ~/dev_ws/src/jmu-lunabotics
cd ~/dev_ws/src/jmu-lunabotics
```

Run the appropriate command once per supported device from that repository
root. The installer is explicit and requests confirmation before it changes
the host. It needs an internet connection for Ubuntu/ROS packages and `sudo`
permission on the device.

```bash
# Ubuntu 22.04 (Jammy) development host
./scripts/bootstrap_dev.sh --install --profile development

# Verified Jetson Orin Nano running the locked JetPack / L4T image
./scripts/bootstrap_dev.sh --install --profile jetson
```

The canonical layout writes build products to `~/dev_ws/build`,
`~/dev_ws/install`, and `~/dev_ws/log`. A standalone clone is accepted for CI
or a deliberate local exception: pass its repository root with
`--workspace "$(pwd)"`; its build products remain inside that repository.

The installer is suitable to rerun when repairing a partially provisioned
machine or intentionally updating its declared dependencies, but it is not a
normal start-of-day command. After the initial provisioning, use
`./scripts/build.sh` and `./scripts/test.sh` when repository code changes.

Without `--install`, bootstrap defaults to the non-mutating readiness check:

```bash
./scripts/bootstrap_dev.sh --check
```

Run that check after provisioning and whenever a device's software state is in
doubt. A failure before provisioning is expected and should identify the
missing prerequisite rather than make an unrequested change.

On a supported device, a successful installer has already run the repository
build and test suite, followed by the same readiness check. It cannot source
the environment into the terminal that launched it; use the activation helper
below afterward.

If a ROS apt source is already configured for a different Ubuntu codename, the
installer stops before installing ROS packages. Repair or remove that stale
source deliberately; it does not mix Jammy and non-Jammy ROS repositories.

## Every new terminal

ROS environment variables do not survive a new terminal. After the one-time
installation and an initial workspace build, source the tracked activation
helper from the repository root:

```bash
source scripts/activate.sh
```

This sources the locked ROS distribution and the enclosing workspace overlay
for the current shell only. It deliberately does not edit `.bashrc`, another
shell profile, or global system startup files.

The helper rejects a non-Humble underlay by default. An existing Jazzy host may
be used only for a clearly labelled structural-only or local bench check by
making both overrides explicit before sourcing it:

```bash
export LUNABOT_ROS_DISTRO=jazzy
export LUNABOT_ALLOW_NON_TARGET_ACTIVATION=1
source scripts/activate.sh
```

That opt-in does not alter the target lock or provide Jetson/ZED validation.

## Intentional boundaries

Bootstrap may install the repository's supported Ubuntu/ROS development
dependencies, resolve declared package dependencies, and prepare the local
workspace. It intentionally does **not**:

- flash, upgrade, or otherwise modify JetPack, L4T, NVIDIA firmware, CUDA, or
  the Jetson base image;
- install or validate the ZED SDK or ZED ROS 2 wrapper;
- start a motor, access a motor controller, launch robot hardware, or validate
  a physical propulsion or safety path;
- prove a connected webcam, AprilTag, ZED, calibration, camera extrinsic, or
  field-localization result; or
- edit persistent shell configuration.

Consequently, a successful bootstrap makes the software dependencies available
for mock development and, when a connected webcam passes its own preflight,
the non-actuating webcam-AprilTag bench. It does not certify a physical robot,
the ZED path, target performance, or competition readiness. Follow the
[operations guide](operations.md), the [platform lock](resources/software_platform_lock.md),
and the ordered hardware/safety phases before using physical hardware.
