"""Host-independent contract checks for the one-time developer bootstrap."""

from __future__ import annotations

import re
import subprocess
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
BOOTSTRAP_SCRIPT = REPOSITORY_ROOT / "scripts" / "bootstrap_dev.sh"
ACTIVATION_SCRIPT = REPOSITORY_ROOT / "scripts" / "activate.sh"


def _bootstrap_source() -> str:
    assert BOOTSTRAP_SCRIPT.is_file(), f"missing bootstrap script: {BOOTSTRAP_SCRIPT}"
    return BOOTSTRAP_SCRIPT.read_text(encoding="utf-8")


def test_bootstrap_help_exposes_the_one_time_setup_interface() -> None:
    """Discoverable setup must not require a target ROS installation to inspect."""
    result = subprocess.run(
        ["bash", str(BOOTSTRAP_SCRIPT), "--help"],
        cwd=REPOSITORY_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr
    for option in ("--check", "--install", "--profile", "--workspace", "--dry-run"):
        assert option in result.stdout
    assert re.search(r"Default (?:behavior|action).*--check", result.stdout, re.IGNORECASE)


def test_bootstrap_defaults_to_a_non_mutating_check() -> None:
    """Installation remains explicit so an ordinary invocation cannot change a host."""
    source = _bootstrap_source()

    assert re.search(r'(?m)^\s*(?:local\s+)?ACTION\s*=\s*["\']?check["\']?', source)
    assert re.search(r"--install(?:\||\))", source)
    assert "run_check" in source


def test_bootstrap_rejects_conflicting_actions_before_host_work() -> None:
    """Argument order must not turn a read-only invocation into an install."""
    result = subprocess.run(
        ["bash", str(BOOTSTRAP_SCRIPT), "--check", "--install"],
        cwd=REPOSITORY_ROOT,
        check=False,
        capture_output=True,
        text=True,
    )

    assert result.returncode != 0
    assert "conflicting actions" in result.stderr


def test_installer_uses_humble_and_rosdep_as_the_dependency_authority() -> None:
    """Packages come from manifests rather than a stale hand-maintained list."""
    source = _bootstrap_source()

    assert 'TARGET_ROS_DISTRO="humble"' in source
    assert "rosdep install" in source
    assert "--from-paths" in source
    assert "--ignore-src" in source
    assert re.search(
        r"ros-(?:humble|\$\{?TARGET_ROS_DISTRO\}?)-ros-base",
        source,
    )

    installer_start = source.index("install_repository_environment()")
    ros_source_index = source.index("  configure_ros_apt_source\n", installer_start)
    host_install_index = source.index(
        '  run_sudo apt-get install -y --no-install-recommends "${host_packages[@]}"',
        installer_start,
    )
    assert ros_source_index < host_install_index


def test_bootstrap_validates_the_workspace_before_provisioning() -> None:
    """A misplaced repository must fail before privileged installation work begins."""
    source = _bootstrap_source()

    assert re.search(r"--workspace(?:\||\))", source)
    assert "WORKSPACE_ROOT" in source
    assert re.search(r"validate_[a-z_]*workspace[a-z_]*\s*\(\)", source)
    assert re.search(r"-d\s+[\"']?\$\{?WORKSPACE_ROOT\}?", source)
    assert "/src" in source


def test_activation_helper_is_source_only_and_loads_both_overlays() -> None:
    """Future terminals get explicit underlay and workspace activation without .bashrc edits."""
    assert ACTIVATION_SCRIPT.is_file(), f"missing activation helper: {ACTIVATION_SCRIPT}"
    source = ACTIVATION_SCRIPT.read_text(encoding="utf-8")

    assert "BASH_SOURCE[0]" in source
    assert '"$0"' in source or "${0}" in source
    assert re.search(r"\bsource\b|(^|\s)\.\s", source)
    assert "/opt/ros/" in source
    assert "install/setup.bash" in source


def test_bootstrap_makes_target_specific_omissions_explicit() -> None:
    """JetPack and ZED setup stay outside unattended host provisioning."""
    source = _bootstrap_source().lower()

    assert "jetpack" in source
    assert "zed" in source
