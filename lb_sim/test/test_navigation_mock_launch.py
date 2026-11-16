"""Exercise launch preflight without starting devices or ROS processes."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import LogInfo
import pytest


ROOT = Path(__file__).parents[2]


@pytest.fixture(
    params=["lb_sim/launch/nav2_mock.launch.py", "lb_navigation/launch/navigation_mock.launch.py"]
)
def profile(request):
    spec = importlib.util.spec_from_file_location("mock_launch_preflight", ROOT / request.param)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def context(**overrides):
    instance = LaunchContext()
    instance.launch_configurations.update(
        {
            "enable_nav2": "true",
            "allow_mock_navigation": "true",
            "allow_test_only_tf": "true",
            "use_mock_hardware": "true",
            "enable_motors": "false",
        }
    )
    instance.launch_configurations.update(overrides)
    return instance


def test_disabled_profile_starts_no_processes(profile):
    actions = profile._launch_setup(context(enable_nav2="false"))
    assert len(actions) == 1 and isinstance(actions[0], LogInfo)


@pytest.mark.parametrize(
    "overrides",
    [
        {"allow_mock_navigation": "false"},
        {"use_mock_hardware": "false"},
        {"enable_motors": "true"},
        {"enable_nav2": "yes"},
    ],
)
def test_unsafe_combinations_fail_before_nodes_are_constructed(profile, overrides):
    with pytest.raises(RuntimeError):
        profile._launch_setup(context(**overrides))
