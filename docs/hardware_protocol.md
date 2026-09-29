# Hardware protocol

No motor or microcontroller packet protocol has been selected. Phase 2 does
not infer CAN IDs, bitrate, packet fields, pinouts, controller modes, or fault
semantics from the BOM.

## Phase 2 abstract boundary

`lb_hardware` defines `DriveTransport` with `connect`, `disconnect`,
`read_state`, `write_command`, `set_enabled`, `stop`, and `get_faults`. Wheel
commands and state use radians and radians/second at this boundary.
`MockDriveTransport` supports software-only testing. `RealDriveTransport` is
an intentionally unavailable skeleton: it opens no device and returns failure
for every operation until `DRIVE-03` and `SAFE-01` are closed.

This is a transport abstraction, not a protocol selection. In particular, the
listed ODrive USB-to-CAN adapter does not resolve whether the Jetson talks to a
microcontroller, whether a microcontroller talks to SPARK MAX controllers, or
whether the controller topology is two or four drive channels.

The future drive interface must document:

- Jetson, USB-to-CAN, microcontroller, and motor-controller topology.
- Transport and framing, message units, command rate, heartbeat, timeout,
  acknowledgement, and reconnection behavior.
- Encoder source, scaling, sign convention, and timestamp source.
- Fault/status fields and the explicit enable, stop, reset, and disable paths.
- The independent physical emergency-stop path.

All values at the ROS boundary must use SI units. The preconditions for work
in this area are tracked in [the TBD register](tbd_register.md).
