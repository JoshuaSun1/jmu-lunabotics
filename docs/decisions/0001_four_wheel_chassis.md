# ADR 0001: Four-wheel drive chassis scope

**Date:** 2026-08-10
**Status:** Accepted

## Context

The BOM contains six NEO 2.0 motors and six SPARK MAX controllers. That count
does not establish the number of drive wheels or the drivetrain kinematics.

## Evidence

The team confirmed that the robot will use four driven wheels with skid/tank
steering and a large scooping bucket. The user-provided
[`Current Lunabot Robot Design Codex Handoff`](../resources/Current_Lunabot_Robot_Design_Codex_Handoff.md)
records the same current baseline. The remaining ordered motors are intended
for the digging arm rather than extra drive wheels.

## Decision

Model the robot as a four-wheel skid-steer chassis. Do not model the BOM as a
six-wheel drivetrain and do not create steering joints. This decision only
selects the kinematic topology; it does not establish wheel positions, wheel
radius, track width, drivetrain reduction, encoder signs, or odometry values.

## Consequences

- Phase 1 provides a synthetic four-wheel and conceptual-bucket frame tree
  for mock TF validation, with four continuous wheel joints.
- Physical wheel positions, wheel dimensions, bucket envelope, frame origin,
  sensor transforms, and drive-controller configuration remain unmeasured
  parameters/TBDs.
- `lb_hardware` must not select a real transport or calculate odometry until
  the calibration and protocol records are complete. `diff_drive_controller`
  configuration remains a Phase 2 task.

## Follow-up and validation

`DRIVE-01` is closed by the team confirmation and the design handoff. Close
`GEOM-01` and `DRIVE-02` using a reviewed CAD model or measured chassis record.
Validate the Phase 1 mock frame tree with its automated launch test; that test
does not validate physical kinematics or geometry.
