# Nav2 crash recovery and N4 headless runtime evidence — 2026-11-15

Base revision: `0605ea4` (`saving state after crash`), branch `sun-nav2-1-N4`.
This follow-up is uncommitted; no new Git commit was created.
Host: Ubuntu 24.04, x86_64, ROS 2 Jazzy. Humble/Jetson remains unvalidated.

## Recovery changes

- Restored script installation for fused-pose pass-through, development TF
  bridge, and navigation command gate, plus the safety configuration directory.
- Registered all saved N0–N4 unit, contract, and runtime tests in CMake.
- Declared the scripts' message, Python, launch, cross-package and test
  dependencies; added standalone costmaps to bootstrap readiness checks.
- Fixed local costmap width/height YAML types: Nav2 requires integer meters.
- Disabled lifecycle bond monitoring for this standalone N4 profile. The live
  manager otherwise failed waiting for a bond from the active global costmap.
  [Upstream Costmap2DROS](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_costmap_2d/src/costmap_2d_ros.cpp)
  confirms standalone activation does not create a lifecycle bond.
- Defaulted N4 to explicit lifecycle startup after the test TF is current,
  avoiding a startup race against the existing 2.0 s initial TF timeout.
- Bounded synthetic test input to 20 Hz, added grid/footprint/command-loss
  assertions, and shut down lifecycle nodes before terminating the launch
  process group. No planner/controller or final drive connection was added.

## Clean build verification

From `/home/jsun3/dev_ws`, using a new build/install tree rather than the
pre-crash artifacts:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base /tmp/lb_crash_recovery/log build \
  --build-base /tmp/lb_crash_recovery/build \
  --install-base /tmp/lb_crash_recovery/install \
  --symlink-install --packages-up-to lb_navigation lb_launch
```

Result: eight packages built successfully. The normal workspace was also
rebuilt successfully for `lb_localization lb_safety lb_navigation lb_launch`.
The clean overlay supplies the installed executable paths and package shares
used by the subprocess runtime test.

## Final affected-package test run

```bash
source /opt/ros/jazzy/setup.bash
source /tmp/lb_crash_recovery/install/local_setup.bash
ROS_LOG_DIR=/tmp/lb_crash_recovery/final_ros_logs ROS_DOMAIN_ID=79 \
  colcon --log-base /tmp/lb_crash_recovery/log test \
  --build-base /tmp/lb_crash_recovery/build \
  --install-base /tmp/lb_crash_recovery/install \
  --packages-select lb_localization lb_safety lb_navigation lb_launch \
  --return-code-on-test-failure
colcon test-result --test-result-base /tmp/lb_crash_recovery/build --verbose
```

Result: all four selected packages completed successfully; test-result summary
reported **127 tests, zero errors, zero failures, zero skips**. The N4 runtime
test passed rather than skipping. Local DDS networking permission was needed:
the restricted sandbox denies UDP sockets and cannot provide this evidence.

Retained [runtime output](2026-11-15-nav2-crash-recovery/nav2_n4_lifecycle_runtime.txt)
and [JUnit result](2026-11-15-nav2-crash-recovery/nav2_n4_lifecycle_runtime.xunit.xml).
Original process logs remain in `/tmp/lb_crash_recovery/final_ros_logs`.

## What the live N4 test proves

- Explicit startup succeeds and map server/global costmap/local costmap are active.
- Global map: `map`, 139 x 102 cells at 0.05 m/cell; local rolling map: `odom`,
  80 x 80 cells at 0.05 m/cell.
- Both published footprints match the 1.10 x 0.60 m rectangle at the test pose.
- Published occupancy contains lethal and inflated cells. The live raw
  `GetCostmap` service returns lethal perimeter cost 254, adjacent inflation,
  and zero central free-space cost.
- A synthetic 0.2 m/s command passes while valid. Stopping pose input yields
  invalid localization and only zero gated commands despite continued input.
- No final `/cmd_vel` publisher is present.

## Remaining acceptance boundaries

This completes headless N4 runtime validation on this host, not all Nav2
acceptance. RViz graphical inspection remains pending. No planner exists yet,
so demonstrating that planned paths avoid occupied cells belongs to N5/N6.
No live footprint-collision service result, physical collision clearance,
rolling-window motion trial, performance distribution, or endurance is claimed.

The manager's disabled bonds mean N4 does not monitor process health through
that mechanism. Costmaps can remain active with retained data after tag loss;
the independently tested command gate is the lock. N5 must establish suitable
server/command-path monitoring before mock navigation integration. All
geometry, inflation, timing and limits retain their development assumptions.
Physical use remains prohibited; there is no live obstacle source, continuous
localization, physical safety supervisor, or motor connection.
