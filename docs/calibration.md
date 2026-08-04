# Calibration

Calibration is deferred until mechanical measurements and sensors are
available. No numeric calibration values belong in source code.

## Required records

- `base_link` origin, axes, and all rigid sensor mounting transforms.
- Wheel radius, effective wheel separation, gearing, encoder resolution, and
  wheel/encoder sign conventions.
- Robot footprint polygons for every mechanism configuration.
- ZED Mini calibration verification, USB topology, timestamp behavior, and
  frame authority.
- LiDAR mounting transform and chassis/mechanism exclusion sectors.
- Arena map origin, axes, occupancy thresholds, and AprilTag pose procedure.
- AprilTag family, ID, physical size, pose uncertainty, and placement.

Each record must include units, measurement method, date, operator, uncertainty,
and the configuration file that consumes it. See `docs/tbd_register.md` for
the current unresolved list.
