"""Contract tests for the Phase 0 repository scaffold."""

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
PACKAGE_NAMES = (
    "lb_interfaces",
    "lb_model",
    "lb_hardware",
    "lb_localization",
    "lb_sensors",
    "lb_navigation",
    "lb_state_manager",
    "lb_safety",
    "lb_launch",
    "lb_sim",
)


def test_required_packages_have_matching_manifests() -> None:
    """Every package in the required architecture has a matching manifest."""
    for package_name in PACKAGE_NAMES:
        manifest = REPOSITORY_ROOT / package_name / "package.xml"
        assert manifest.is_file(), f"missing package manifest: {manifest}"
        assert f"<name>{package_name}</name>" in manifest.read_text(encoding="utf-8")


def test_phase_zero_resources_and_version_lock_exist() -> None:
    """The scaffold retains the specification and its decision inputs."""
    required_paths = (
        "README.md",
        "docs/lunabotics_autonomy_software_spec.md",
        "docs/resources/Master Progressive BOM - MASTER.pdf",
        "docs/resources/software_platform_lock.md",
        "docs/tbd_register.md",
        "docs/implementation_log.md",
        "resources/devlog.md",
    )
    for relative_path in required_paths:
        assert (REPOSITORY_ROOT / relative_path).is_file(), f"missing: {relative_path}"


def test_target_runtime_lock_is_explicit() -> None:
    """Native bootstrap and the platform record retain the proposed target lock."""
    bootstrap = (REPOSITORY_ROOT / "scripts" / "bootstrap_dev.sh").read_text(
        encoding="utf-8"
    )
    platform_lock = (
        REPOSITORY_ROOT / "docs" / "resources" / "software_platform_lock.md"
    ).read_text(encoding="utf-8")
    assert 'TARGET_ROS_DISTRO="humble"' in bootstrap
    assert 'TARGET_UBUNTU_VERSION="22.04"' in bootstrap
    assert 'TARGET_L4T_RELEASE="R36 (release), REVISION: 5.0"' in bootstrap
    assert "JetPack | 6.2.2" in platform_lock
    assert "ZED SDK | 5.2.3" in platform_lock
