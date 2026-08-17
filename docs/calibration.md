# Calibration

Calibration is deferred until mechanical measurements and sensors are
available. Numeric calibration values must live in documented configuration
records rather than source code, with explicit provenance and validation
status.

The numeric values in `lb_model/config/mock_geometry.yaml` are an explicitly
synthetic Phase 1 visual/test profile, not calibration data. Preliminary values
in the mechanical-design handoff must not be copied into controller, odometry,
navigation, safety, or physical-description configuration until reviewed and
measured.

## Phase 4.1 webcam record

`lb_sensors/config/webcam_calibration.yaml` is the narrow exception to the
otherwise deferred sensor-calibration work. It contains a user-confirmed,
archived calibration for the same Logitech UVC webcam at **640 x 480** with a
`plumb_bob` camera model. The calibration is used only by the non-actuating
Phase 4.1 webcam AprilTag bench pipeline:

```text
v4l2_camera -> image_proc/rectify_node -> apriltag_ros
```

Its numerical K, D, R, and P values were copied unchanged from
`old-lunabotics/config/webcam_calibration.yaml`. The only modified field is the
metadata `camera_name`, renamed from `narrow_stereo` to
`uvc_camera_(046d:0825)` to match the confirmed V4L2 camera name; that is not a
new calibration. The launch uses a fail-closed `v4l2-ctl` preflight to set and
verify the 15 Hz source rate before `v4l2_camera` opens the 640 x 480 stream.

The visible bench tag is `tag36h11` ID 0. Its user-confirmed physical size is
0.250 m from detector corner to detector corner, not the outside edge of the
paper. `max_hamming: 0` is the Phase 4.1 detector baseline. These values make a
camera-relative observation possible, but do not establish metric accuracy,
uncertainty, camera timestamp behavior, tag world pose, or robot pose.

Before any localization use, repeat and record a measured-distance/angle
validation, calibration date/operator/uncertainty, physical mount survey, and
the final owner of `base_link -> webcam_optical_frame`. Keep the capture
profile at 640 x 480 until a different profile has its own calibration record.
The synthetic webcam transform in the Phase 1 model is not a physical extrinsic
and must not be used for this validation.

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
