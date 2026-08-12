from __future__ import annotations

import re
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server"))

from vehicle_dynamics import VEHICLE_PRESETS  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "maxscript" / "VehicleRigTool.ms"

GENERATED_NODES = (
    "VRIG_Master",
    "VRIG_Body",
    "VRIG_Steering",
    "VRIG_PathGuide",
    "VRIG_Wheel_FL",
    "VRIG_Wheel_FR",
    "VRIG_Wheel_RL",
    "VRIG_Wheel_RR",
)

PRESET_FIELDS = {
    "wheelbase": "wheelbase",
    "track": "track",
    "tyreRadius": "tyre_radius",
    "topSpeed": "top_speed_mps",
    "acceleration": "acceleration_mps2",
    "braking": "braking_mps2",
    "lateralGrip": "lateral_grip_g",
    "maxSteer": "max_steer_deg",
    "springTravel": "spring_travel_m",
    "dampingRatio": "damping_ratio",
    "maxRoll": "max_roll_deg",
    "maxPitch": "max_pitch_deg",
}

_KEYWORD = re.compile(r'(\w+):(?:"([^"]*)"|(-?\d+\.?\d*))')


@pytest.fixture(scope="module")
def source() -> str:
    return SCRIPT.read_text(encoding="utf-8")


def _parse_presets(source: str) -> dict[str, dict[str, object]]:
    presets: dict[str, dict[str, object]] = {}
    for line in source.splitlines():
        stripped = line.strip()
        # Only preset declarations carry a quoted literal name; the copy
        # helper passes fields through by reference and is not a preset.
        if not stripped.startswith('VehicleRigProfile name:"'):
            continue
        fields: dict[str, object] = {}
        for key, text, number in _KEYWORD.findall(stripped):
            fields[key] = text if number == "" else float(number)
        presets[str(fields["name"])] = fields
    return presets


def test_maxscript_exposes_required_safety_actions(source: str) -> None:
    for token in (
        "Create Vehicle Rig",
        "Preflight",
        "Bake Animation",
        "Remove Generated Rig",
        "VRIG_Master",
    ):
        assert token in source


def test_script_limits_deletion_to_generated_nodes(source: str) -> None:
    assert 'matchPattern node.name pattern:"VRIG_*"' in source
    assert "originalParent" in source
    assert "originalTransform" in source


def test_script_declares_every_generated_node(source: str) -> None:
    for name in GENERATED_NODES:
        assert name in source


def test_presets_cover_the_python_vehicle_presets(source: str) -> None:
    assert set(_parse_presets(source)) == set(VEHICLE_PRESETS)


@pytest.mark.parametrize("preset_name", sorted(VEHICLE_PRESETS))
def test_preset_values_match_the_python_source_of_truth(
    source: str, preset_name: str
) -> None:
    script_preset = _parse_presets(source)[preset_name]
    profile = VEHICLE_PRESETS[preset_name]
    for script_field, profile_field in PRESET_FIELDS.items():
        assert script_preset[script_field] == pytest.approx(
            getattr(profile, profile_field)
        ), f"{preset_name}.{script_field} drifted from vehicle_dynamics"


def test_script_uses_layered_controllers_for_manual_edits(source: str) -> None:
    assert "position_list()" in source
    assert "rotation_list()" in source


def test_script_never_performs_prohibited_host_operations(source: str) -> None:
    forbidden = (
        "saveMaxFile",
        "loadMaxFile",
        "resetMaxFile",
        "System.Net",
        "downloadFile",
        "freeCamera",
        "targetCamera",
        "omniLight",
        "directionalLight",
    )
    lowered = source.lower()
    for token in forbidden:
        assert token.lower() not in lowered, f"{token} is out of scope for this tool"


def test_script_renders_nothing(source: str) -> None:
    assert not re.search(r"\brender\s*\(", source, flags=re.IGNORECASE)


def test_preflight_reports_every_python_issue_code(source: str) -> None:
    for code in (
        "too_short",
        "zero_length_segment",
        "duplicate_knot",
        "self_intersection",
        "sharp_kink",
        "steering_limit",
    ):
        assert code in source
