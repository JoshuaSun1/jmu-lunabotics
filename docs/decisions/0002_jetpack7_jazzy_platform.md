# ADR 0002: JetPack 7 and ROS 2 Jazzy robot baseline

**Date:** 2026-10-13
**Status:** Accepted as a software baseline; target validation open
**Owner:** JMU Lunabotics software team

## Context

The repository originally targeted JetPack 6.2.2, Ubuntu 22.04, and ROS 2
Humble. The team reports that the robot actually runs JetPack 7.2.1 on Ubuntu
24.04 with CUDA 13 and ZED SDK 5.5. Retaining the old provisioning guards
would reject the robot and build against a different ROS distribution.

## Decision

Use ROS 2 Jazzy, the supported Ubuntu 24.04 arm64 distribution. Record the
JetPack 7.2.1 software mapping as L4T 39.2.1 and CUDA 13.2.2, and select ZED
SDK 5.5.0 with ZED ROS 2 wrapper tag `v5.5.0` as the repository baseline.
The user reported major SDK/CUDA versions; the exact installed patches require
verification on the Jetson.

## Consequences

- Bootstrap, build, test, activation, container scaffold, and CI use Jazzy.
- The generic Ubuntu 24.04 container checks ROS package builds but does not
  supply JetPack, CUDA, or the ZED SDK.
- A previous Humble build or overlay must be rebuilt under Jazzy.
- The current software contains no ZED launch path and no physical drive path.

## Validation

NVIDIA lists JetPack 7.2.1 for Orin with L4T 39.2.1, Ubuntu 24.04, and CUDA
13.2.2. ROS publishes Jazzy packages for Ubuntu 24.04 arm64. Stereolabs
publishes ZED SDK 5.5 for JetPack 7.2 and wrapper `v5.5.0` with Jazzy support.
Physical ZED Mini streaming, exact installed package versions, ros2_control
runtime behavior, and performance remain to be tested on the robot.

See [the platform lock](../resources/software_platform_lock.md) for sources
and the ordered validation procedure.
