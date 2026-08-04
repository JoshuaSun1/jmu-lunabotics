"""Contract tests for the Phase 0 repository scaffold."""

from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
PACKAGE_NAMES = (
    "lunabot_interfaces",
    "lunabot_description",
    "lunabot_hardware",
    "lunabot_localization",
    "lunabot_perception",
    "lunabot_navigation",
    "lunabot_mission",
    "lunabot_safety",
    "lunabot_bringup",
    "lunabot_sim",
)


def test_required_packages_have_matching_manifests() -> None:
    """Every package in the required architecture has a matching manifest."""
    for package_name in PACKAGE_NAMES:
        manifest = REPOSITORY_ROOT / "src" / package_name / "package.xml"
        assert manifest.is_file(), f"missing package manifest: {manifest}"
        assert f"<name>{package_name}</name>" in manifest.read_text(encoding="utf-8")


def test_phase_zero_resources_and_version_lock_exist() -> None:
    """The scaffold retains the specification and its decision inputs."""
    required_paths = (
        "README.md",
        "docker/versions.env",
        "docs/lunabotics_autonomy_software_spec.md",
        "docs/resources/Master Progressive BOM - MASTER.pdf",
        "docs/resources/software_platform_lock.md",
        "docs/tbd_register.md",
        "docs/implementation_log.md",
    )
    for relative_path in required_paths:
        assert (REPOSITORY_ROOT / relative_path).is_file(), f"missing: {relative_path}"


def test_target_runtime_lock_is_explicit() -> None:
    """The target lock remains machine-readable and avoids a runtime-image guess."""
    values = (REPOSITORY_ROOT / "docker" / "versions.env").read_text(encoding="utf-8")
    assert "TARGET_PLATFORM=linux/arm64" in values
    assert "ROS_DISTRO=humble" in values
    assert "JETPACK_VERSION=6.2.2" in values
    assert "L4T_VERSION=36.5.0" in values
    assert "ZED_SDK_VERSION=5.2.3" in values
    assert "JETSON_RUNTIME_IMAGE=TBD_AFTER_TARGET_VALIDATION" in values
