# JMU NASA Lunabotics: Current Robot Design Handoff

**Document purpose:** Provide Codex with the current physical robot design baseline for continued CAD, mechanical, electrical, software, and systems-integration work.

**Source basis:** Project design documents, decision matrices, meeting notes, BOM, scoop calculations, concept drawings, and the May 2026 design presentation contained in the NASA Lunabotics project archive.

**Design maturity:** Preliminary architecture selected; detailed design is not yet finalized. Treat the design below as the current baseline, not as a fabrication-ready specification.

## 1. Project objective

Design and build a semi-autonomous mining robot for NASA Lunabotics. The robot must traverse BP-1-like regolith, avoid obstacles, excavate and retain regolith, transport it, and deposit it to form a berm. The design should minimize mass, dust generation, energy use, operational complexity, and dependence on teleoperation.

## 2. Current system concept

The selected robot is a four-wheeled, skid-steer vehicle built around a welded aluminum-tube frame. A front scoop performs excavation and serves as the primary regolith container. Articulated arms and linear actuators raise, lower, and dump the scoop. Large custom wheels provide traction in loose regolith. Electrical equipment and batteries are mounted primarily within or toward the rear of the chassis to protect them and counterbalance a loaded scoop.

The selection documents support the following decisions:

- Front scoop selected as the excavation method.
- Scoop itself used as the main storage volume.
- Skid/tank steering selected as the drivetrain configuration.
- Four large, non-rubber regolith wheels.
- Actuator-driven scoop lift and dumping.
- Aluminum rectangular tubing for the primary frame.
- Brushless electric motors with geared and chained drivetrain reduction.

## 3. Mechanical architecture

### 3.1 Frame

Current concept:

- Welded aluminum rectangular-tube chassis.
- Approximately 384 inches of tubing was estimated for the preliminary design.
- Open structural layout in the current CAD, with internal mounting space for the battery, power distribution, motor controllers, computer, router, and other electronics.
- Frame must support the robot, a loaded scoop, drivetrain loads, actuator loads, lifting loads, and torsion from skid steering.
- Electronics require dust shielding even if the structural frame remains relatively open.
- The robot must include four clearly marked lifting points capable of supporting the full robot mass.

The exact tubing alloy, cross-section, wall thickness, member lengths, joint geometry, weld design, equipment tray design, lifting-point design, and enclosure panels are not finalized.

### 3.2 Excavation and storage

The selected excavation system is a front scoop. The current intent is for the scoop to:

- Penetrate loose regolith as the robot drives forward.
- Retain the collected material during transport.
- Act as the primary storage container instead of transferring material to a separate bin.
- Raise to a suitable dumping position at the berm area.
- Deposit material in a controlled manner while limiting dust generation.

Current preliminary calculation inputs include:

| Parameter | Preliminary value |
|---|---:|
| Scoop width | 0.75 m |
| Scoop depth | 0.25 m |
| Scoop height | 0.25 m |
| Front scoop-arm length | 0.50 m |
| Rear scoop-arm length | 0.50 m |
| Calculated scoop volume | 0.0234375 m^3 |
| Assumed regolith density | 1,860 kg/m^3 |
| Theoretical full regolith load | 43.59 kg |

These values are design-study inputs, not finalized dimensions. In particular, a theoretical 43.59 kg payload may be impractical when combined with the robot mass, 80 kg competition limit, stability, actuator capacity, and traction. Codex must not silently treat theoretical scoop capacity as the desired operating payload.

### 3.3 Scoop linkage and actuators

The May 2026 presentation describes:

- Two linear actuators driving the scoop lifting motion.
- A third actuator controlling the dumping mechanism.
- Two articulated structural arms connecting the scoop to the chassis.

Earlier concept drawings show a moving rear wall inside the scoop that pushes regolith out. Other descriptions imply that the scoop may rotate or tilt for dumping. Therefore, the exact dumping architecture remains unresolved.

Candidate actuators considered by the team were 12 V units with position feedback and approximately 12-16 inches of stroke. Candidate force ratings ranged from roughly 220 lbf to 450 lbf, while the scoop calculation sheet contains a 3,000 N study value. The team has not yet documented a validated actuator selection based on linkage geometry and worst-case loads.

Detailed design must determine:

- Scoop pivot positions.
- Arm lengths and cross-sections.
- Lift-actuator mounting points and stroke.
- Dump-actuator mounting and mechanism.
- Digging, transport, and dumping angles.
- Mechanical stops.
- Pin and bearing sizes.
- Structural loads through the complete motion range.
- Required actuator force, including friction and safety factors.
- Whether position feedback is necessary for control and fault detection.
- Center-of-mass movement with an empty and loaded scoop.

### 3.4 Wheels and mobility

The robot uses four custom wheels and skid/tank steering. The original wheel decision matrix favored ribbed wheels, while later CAD work includes a herringbone-wheel part and the current robot rendering shows chevron-like tread. The final wheel geometry is therefore not yet locked.

Current wheel-study assumptions include:

- Wheel radius approximately 0.25 m.
- Large diameter and low ground pressure for loose regolith.
- No rubber pneumatic tires or foam-filled tires.
- Potential aluminum hub with printed nylon tread, or another hybrid construction.
- Tread must provide forward traction without excessive sinkage, excavation, lateral slip, or dust generation.

Final wheel selection requires full-scale testing in representative material. The team should measure sinkage, slip, current draw, turning resistance, obstacle performance, durability, and manufacturability.

### 3.5 Drivetrain

The current drivetrain concept uses one powered side on the left and one on the right, enabling differential/skid steering. Preliminary research assumes two main drivetrain motors with chain distribution to front and rear wheels.

Working drivetrain concept:

- NEO 2.0 brushless motors.
- Spark MAX motor controllers.
- Approximately 5:1 planetary reduction at each drivetrain motor.
- Approximately 10:1-15:1 additional chain/sprocket reduction.
- Approximately 50:1-75:1 total reduction.
- Preliminary target of approximately 10 N m per driven side before the final safety-factor and terrain analysis.

The reduction values are preliminary. Codex should preserve them as hypotheses until robot mass, wheel radius, target speed, rolling resistance, grade, sinkage, turning resistance, motor curves, controller limits, and drivetrain efficiency are known.

## 4. Electrical and computing architecture

The progressive BOM and design documents identify the following baseline components:

| Component | Current role/status |
|---|---|
| NVIDIA Jetson Orin Nano | Onboard autonomy computer; listed as ordered |
| Stereolabs ZED Mini | Depth camera and IMU; listed as ordered |
| Logitech C270 webcam | AprilTag/video camera; listed as ordered |
| TP-Link Omada router | Robot/MCC network; listed as ordered |
| NEO 2.0 brushless motors | Drivetrain and/or mechanisms; six planned |
| Spark MAX controllers | Six planned for motor control |
| REV Power Distribution Hub | Main protected power distribution; listed as ordered |
| 12 V, 18 Ah SLA battery | Proposed main robot battery |
| 120 A circuit breaker | Main overcurrent protection; listed as ordered |
| Physical E-stop | Competition-required shutdown; listed as ordered |
| PowerWerx power analyzer/logger | Competition-required power logging; listed as ordered |
| ODrive USB-to-CAN adapter | Jetson-to-motor-controller CAN connection; listed as ordered |

The current electrical block diagram is conceptual. A final schematic, wire sizing, fuse selection, grounding plan, power budget, connector plan, battery restraint, enclosure layout, and emergency-stop implementation are still required.

The documents variously describe five or six motors/powered devices. Do not assume that six identical rotary motors are required: the final count depends on whether scoop functions use linear actuators, rotary motors, or both.

## 5. Sensors and autonomy-related physical provisions

The planned localization and perception sensor stack is:

- Wheel odometry.
- ZED Mini IMU.
- AprilTag detection using the webcam.
- ZED depth sensing for obstacle detection.

Physical design must provide:

- Rigid, vibration-resistant sensor mounts.
- Measurable sensor-to-`base_link` transforms.
- Clear fields of view for both cameras through the required robot motions.
- Protection against impacts and regolith.
- A practical lens-cleaning or dust-mitigation approach.
- Cable routing that does not cross scoop-linkage, wheel, chain, or actuator motion.
- Access for calibration, maintenance, and replacement.
- Camera placement that remains useful when the scoop is raised.

Do not place sensors in CAD solely for visual convenience. Placement must be justified using field of view, expected AprilTag range, obstacle visibility, occlusion, vibration, dust, and transform calibration.

## 6. Governing design constraints

The current project documents identify the following competition constraints. These should be checked against the latest official guidebook before being frozen:

- Stowed envelope: 1.50 m x 0.75 m x 0.75 m, orientation chosen by the team.
- Maximum deployed height: 2.50 m above the regolith surface.
- Maximum total robot mass: 80 kg.
- Four designated lifting points.
- Physical emergency stop.
- Required onboard power meter/data logger.
- Robot must support teleoperation and autonomous operation.
- External power is prohibited during operation.
- GPS is prohibited.
- Rubber pneumatic tires, air/foam-filled tires, open- or closed-cell foam, ultrasonic proximity sensors, and hydraulics are prohibited.
- Excavation may not create downward penetration force greater than the robot weight through anchoring or similar methods.
- Dust generation, mass, energy use, autonomy, and berm performance affect scoring.

## 7. Mass and stability concerns

The April project notes estimate:

- Approximately 16 kg of known components before linear actuators.
- Approximately 22.32 kg before wheels and linear actuators.

These estimates are incomplete and may omit frame details, fasteners, drivetrain transmission, guards, wire, connectors, enclosures, sensor mounts, battery restraints, and manufacturing additions. A formal mass budget should be created with measured or manufacturer-provided masses and a design reserve.

Critical stability cases include:

- Empty scoop lowered during excavation.
- Loaded scoop lowered during travel.
- Loaded scoop raised for dumping.
- Turning with a loaded scoop.
- Driving or stopping on a slope.
- One or more wheels entering uneven terrain.

## 8. Current design status

### Selected or strongly established

- Front-scoop architecture.
- Scoop also functions as the regolith container.
- Four-wheel skid/tank steering.
- Aluminum-tube chassis direction.
- Actuator-driven scoop linkage.
- NEO/Spark MAX motor-control ecosystem.
- Jetson Orin Nano, ZED Mini, webcam, router, E-stop, power logger, and power-distribution approach.

### Preliminary or unresolved

- Final overall dimensions.
- Detailed frame geometry and structural analysis.
- Exact scoop dimensions and payload target.
- Scoop cutting-edge geometry.
- Exact lift linkage and actuator selection.
- Moving-wall versus tilting-scoop dumping.
- Final wheel tread, width, material, and manufacturing method.
- Final gearbox and sprocket ratios.
- Exact motor/actuator count.
- Battery selection and runtime validation.
- Electronics enclosure and cooling/dust strategy.
- Sensor locations and mounts.
- Cable, chain, and pinch-point guards.
- Four lifting-point details.
- Complete mass, center-of-mass, power, and thermal budgets.

## 9. Required next engineering work

1. Confirm all physical constraints against the latest competition guidebook.
2. Establish a configuration-controlled list of selected, provisional, and rejected design decisions.
3. Define a realistic regolith payload target from scoring strategy and the 80 kg robot limit.
4. Complete a parametric scoop/linkage model covering excavation, transport, and dumping positions.
5. Calculate actuator loads across the entire motion range with appropriate safety factors.
6. Select and validate the dumping mechanism.
7. Create a complete mass and center-of-mass model for all operating configurations.
8. Determine drivetrain torque, speed, reduction, chain loading, and motor current from worst-case mobility conditions.
9. Prototype and test full-scale wheels in representative regolith.
10. Complete production CAD with real component envelopes, fasteners, bearings, wiring space, guards, service access, and lifting points.
11. Produce a final electrical schematic and power budget.
12. Define sensor mounts and record all physical coordinate transforms.
13. Perform structural analysis of the frame, scoop arms, pivots, actuator mounts, drivetrain mounts, and lifting points.
14. Conduct subsystem prototypes before freezing the final design.

## 10. Instructions for Codex

When using this document:

- Clearly label every value as **confirmed**, **preliminary**, **assumed**, or **calculated**.
- Do not invent missing dimensions, material properties, performance requirements, or component selections.
- Do not treat the current SolidWorks rendering as fabrication-ready.
- Preserve traceability: record why a design decision was made, its source, date, alternatives, calculations, tests, and downstream effects.
- Flag conflicts between documents instead of silently choosing one.
- Keep mechanical, electrical, software, safety, and competition constraints synchronized.
- When a proposed change affects mass, center of mass, power, sensor visibility, wiring, autonomy, manufacturability, or guidebook compliance, state those effects explicitly.
- Prefer parameterized calculations and CAD-driving dimensions so the design can be updated as test results become available.
- Require verification before promoting a provisional value or component to the frozen baseline.

## 11. One-paragraph baseline summary

The current JMU Lunabotics robot is planned as a four-wheeled skid-steer vehicle with a welded aluminum-tube chassis, large custom regolith wheels, and a front scoop that excavates, stores, transports, and dumps regolith. The scoop is supported by articulated arms, with two linear actuators planned for lifting and a third actuator planned for dumping; however, the final dumping method, linkage geometry, and actuator selection remain unresolved. The drivetrain is expected to use two NEO 2.0 brushless motors, Spark MAX controllers, planetary gearboxes, and chain reduction, while the onboard electrical and autonomy stack includes a 12 V battery system, protected power distribution, Jetson Orin Nano, ZED Mini, webcam, router, E-stop, and required power logger. This is a selected preliminary architecture rather than a completed detailed design, and major remaining work includes payload definition, wheel testing, drivetrain sizing, structural analysis, final CAD, mass and center-of-mass analysis, electrical schematics, sensor placement, dust protection, and compliance verification.
