# Hardware protocol

No motor or microcontroller packet protocol has been selected. Phase 0 must
not infer CAN IDs, bitrate, packet fields, pinouts, controller modes, or fault
semantics from the BOM.

The future drive interface must document:

- Jetson, USB-to-CAN, microcontroller, and motor-controller topology.
- Transport and framing, message units, command rate, heartbeat, timeout,
  acknowledgement, and reconnection behavior.
- Encoder source, scaling, sign convention, and timestamp source.
- Fault/status fields and the explicit enable, stop, reset, and disable paths.
- The independent physical emergency-stop path.

All values at the ROS boundary must use SI units. The preconditions for work
in this area are tracked in [the TBD register](tbd_register.md).
