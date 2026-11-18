"""The real camera profile must not inherit the synthetic robot mount."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node


LAUNCH_FILE = Path(__file__).resolve().parents[1] / "launch" / "bringup.launch.py"


def test_zed_switch_excludes_mock_bench(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    spec = importlib.util.spec_from_file_location("lunabot_bringup", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    entities = module.generate_launch_description().entities

    mock_group = next(entity for entity in entities if isinstance(entity, GroupAction))
    zed_include = next(
        entity
        for entity in entities
        if isinstance(entity, IncludeLaunchDescription)
        and "zed_mini.launch.py" in str(entity.launch_description_source.location)
    )

    for enabled, expect_mock, expect_zed in (
        ("false", True, False),
        ("true", False, True),
    ):
        context = LaunchContext()
        context.launch_configurations.update({"enable_zed": enabled, "enable_apriltags": "false"})
        assert mock_group.condition.evaluate(context) is expect_mock
        assert zed_include.condition.evaluate(context) is expect_zed


def test_depth_switch_requires_zed_and_gates_detector(monkeypatch, tmp_path) -> None:
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path))
    spec = importlib.util.spec_from_file_location("lunabot_bringup", LAUNCH_FILE)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    check = next(
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, OpaqueFunction)
    )
    context = LaunchContext()
    context.launch_configurations.update({"enable_depth_perception": "true", "enable_zed": "false"})
    try:
        check.execute(context)
    except RuntimeError as error:
        assert "requires enable_zed" in str(error)
    else:
        raise AssertionError("depth detector must reject a mock-only launch")

    sensor_launch = LAUNCH_FILE.parents[2] / "lb_sensors" / "launch" / "zed_mini.launch.py"
    spec = importlib.util.spec_from_file_location("zed_mini", sensor_launch)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    detector = next(
        entity
        for entity in module.generate_launch_description().entities
        if isinstance(entity, Node)
    )
    context.launch_configurations["enable_depth_perception"] = "false"
    assert detector.condition.evaluate(context) is False
    context.launch_configurations["enable_depth_perception"] = "true"
    assert detector.condition.evaluate(context) is True


def test_zed_profile_publishes_every_imu_output_the_zed_mini_has() -> None:
    sensors_root = LAUNCH_FILE.parents[2] / "lb_sensors"
    config = (sensors_root / "config" / "zed_mini.yaml").read_text(encoding="utf-8")
    for enabled in ("publish_imu", "publish_imu_raw", "publish_cam_imu_transf"):
        assert f"{enabled}: true" in config
    # The ZED Mini has no magnetometer, barometer, or temperature sensor.
    for absent in ("publish_mag", "publish_baro", "publish_temp"):
        assert f"{absent}: false" in config
    launch_source = (sensors_root / "launch" / "zed_mini.launch.py").read_text(encoding="utf-8")
    assert '"publish_imu_tf": "true"' in launch_source
