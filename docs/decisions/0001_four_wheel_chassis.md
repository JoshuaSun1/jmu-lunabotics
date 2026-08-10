# ADR 0001: Four-wheel drive chassis scope

**Date:** 2026-08-10
**Status:** Partially resolved; steering kinematics remain TBD

## Context

The BOM contains six NEO 2.0 motors and six SPARK MAX controllers. That count
does not establish the number of drive wheels or the drivetrain kinematics.

## Evidence

The team confirmed that the robot will use four driven wheels and a large
scooping bucket, similar in overall function to a bulldozer. The remaining
ordered motors are intended for the digging arm rather than extra drive wheels.

## Decision

Model the physical robot as a four-wheel-drive chassis. Do not model the BOM
as a six-wheel drivetrain.

Do not yet select a skid-steer, differential, or steering-linkage kinematic
model. The visual analogy to a bulldozer is not sufficient evidence for that
software or control decision.

## Consequences

- Phase 1 may provide a synthetic four-wheel and bucket frame tree for mock
  TF validation.
- Physical wheel positions, wheel dimensions, bucket envelope, frame origin,
  sensor transforms, and drive-controller configuration remain unmeasured
  parameters/TBDs.
- `lunabot_hardware` must not select a drive controller or calculate odometry
  until the steering/kinematic model is confirmed.

## Follow-up and validation

Close `DRIVE-01` after a mechanical confirmation of the steering actuation and
turning model. Close `GEOM-01` using a reviewed CAD model or measured chassis
record. No runtime test is applicable to this documentation-only decision.
