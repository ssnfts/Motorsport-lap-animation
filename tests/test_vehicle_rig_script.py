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
    "VRIG_WheelPivot_",
    "VRIG_WheelSpin_",
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


def test_script_exposes_scale_and_forward_axis_controls(source: str) -> None:
    # A model built at a scale other than the scene unit setting, or facing
    # an axis other than +X, must be correctable without editing the source.
    assert "Scene units per metre" in source
    assert "Vehicle faces" in source
    for axis in ('"+X"', '"-X"', '"+Y"', '"-Y"'):
        assert axis in source


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


# Using one of these as an identifier is a load-time syntax error in Max,
# which no amount of offline contract checking would otherwise reveal.
MAXSCRIPT_KEYWORDS = frozenset(
    """
    and animate at by case catch collect continue coordsys do else exit fn for
    from function global if in local mapped not of off on or parameters
    persistent plugin rcmenu return rollout set struct then throw to tool try
    undo utility when where while with
    """.split()
)


def test_script_never_uses_a_maxscript_keyword_as_an_identifier(source: str) -> None:
    declarations = re.findall(r"\blocal\s+([A-Za-z_]\w*)", source)
    declarations += re.findall(r"\bfn\s+([A-Za-z_]\w*)", source)
    clashes = sorted({name for name in declarations if name in MAXSCRIPT_KEYWORDS})
    assert not clashes, f"reserved MaxScript words used as identifiers: {clashes}"


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


def test_script_exposes_non_destructive_import_assembly(source: str) -> None:
    for token in (
        "Auto Assemble Imported Vehicle",
        "vrigAssembleSelectedVehicle",
        "vrigClassifyWheelNode",
        "wheelfl",
        "wheelfr",
        "wheelbl",
        "wheelbr",
    ):
        assert token in source


def test_script_declares_five_path_fixtures(source: str) -> None:
    for name in (
        "VRIG_Test_Simple",
        "VRIG_Test_Complex",
        "VRIG_Test_UpHill",
        "VRIG_Test_DownHill",
        "VRIG_Test_InvalidKnot",
        "EXPECTED REJECT",
    ):
        assert name in source


def test_spline_sampler_uses_the_actual_final_knot(source: str) -> None:
    assert "for index = 0 to sampleCount - 1" in source
    assert "getKnotPoint splineNode 1 (numKnots splineNode 1)" in source


def test_bake_sets_the_requested_animation_range(source: str) -> None:
    assert "animationRange = interval startFrame endFrame" in source


def test_bake_integrates_curvature_for_continuous_heading(source: str) -> None:
    for token in (
        "fn vrigYawDeltaDegrees",
        "local previousYaw = vrigHeadingAtDistance points distances 0.0",
        "local rawYawStep = vrigYawDeltaDegrees curvature travelStep",
        "local yaw = previousYaw + yawStep",
        "previousYaw = yaw",
    ):
        assert token in source


def test_bake_caps_per_frame_yaw_for_unrideable_corner_knots(source: str) -> None:
    for token in (
        "local maxYawStep = 12.0",
        "local yawStep = vrigClamp rawYawStep maxYawStep",
        "local yaw = previousYaw + yawStep",
    ):
        assert token in source


def test_bake_reports_when_sharp_corner_yaw_is_limited(source: str) -> None:
    for token in (
        "local yawLimited = false",
        "if (abs (yawStep - rawYawStep)) > 1e-6 do yawLimited = true",
        "Sharp spline corner was yaw-limited",
    ):
        assert token in source


def test_imported_vehicle_assembly_uses_wheel_centres_and_body_carrier(
    source: str,
) -> None:
    for token in (
        "vrigBoundsCenter",
        "nodeGetBoundingBox",
        "_BodyCarrier",
        "bodyCarrier.parent = root",
        "wheel.position = vrigBoundsCenter wheelSets[wheelIndex]",
    ):
        assert token in source


def test_auto_assembly_infers_vehicle_facing_from_front_and_rear_wheels(
    source: str,
) -> None:
    for token in (
        "fn vrigForwardAxisFromWheels",
        "frontCentre",
        "rearCentre",
        "forwardAxisList.selection = vrigForwardAxisFromWheels boundWheels",
    ):
        assert token in source


def test_wheel_centres_use_evaluated_world_bounds_for_detached_meshes(
    source: str,
) -> None:
    bounds_fn = source[source.index("fn vrigBoundsCenter"):source.index("fn vrigTopAssemblyNode")]
    assert "nodeGetBoundingBox nodes[1] (matrix3 1)" in bounds_fn
    assert "nodeGetBoundingBox nodes[index] (matrix3 1)" in bounds_fn


def test_bake_ground_aligns_imported_meshes_and_limits_suspension_compression(
    source: str,
) -> None:
    for token in (
        "fn vrigGeometryUnder",
        "fn vrigBoundsMinZ",
        "local vehicleFloorZ = vrigBoundsMinZ vehicleGeometry",
        "local groundLift = -vehicleFloorZ",
        "local suspensionDropLimit = amax 0.0 (bodyFloorZ - vehicleFloorZ)",
        "local bodyHeave = amax (response[3] * scale) (-suspensionDropLimit)",
        "masterControllers[1].value = worldPoint + [0, 0, groundLift]",
        "local wheelFloorZ = vrigBoundsMinZ wheelBoundsNodes",
        "local frameFloorZ = vrigBoundsMinZ vehicleBoundsNodes",
        "local wheelBoundsNodes = vrigWheelGroupNodes wheelNodes",
        "masterControllers[1].value = masterControllers[1].value + [0, 0, -frameFloorZ]",
        "Controller writes above are evaluated by Max after their key context",
        "bodyControllers[2].value = slerp bodyControllers[4] fullBodyRotation responseScale",
    ):
        assert token in source


def test_source_nodes_are_bound_before_helper_keys_are_baked(source: str) -> None:
    binding = source.index("rootNode.parent = master")
    bake_loop = source.index("with animate on")
    assert binding < bake_loop


def test_imported_vehicle_assembly_uses_native_groups_for_offset_pivot_meshes(
    source: str,
) -> None:
    for token in (
        "group selection name:",
        "_BodyGroup",
        "WheelGroup_",
        "bodyGroup.parent = bodyCarrier",
        "wheelGroup.parent = wheels[wheelIndex]",
    ):
        assert token in source


def test_wheel_spin_uses_generated_pivots_not_native_group_heads(
    source: str,
) -> None:
    for token in (
        "VRIG_WheelPivot_",
        "VRIG_WheelSpin_",
        "local wheelPivots = #()",
        "local wheelSpins = #()",
        "local wheelPivotControllers = for wheelPivot in wheelPivots collect (vrigLayeredControllers wheelPivot)",
        "local wheelSpinControllers = for wheelSpinNode in wheelSpins collect (vrigLayeredControllers wheelSpinNode)",
        "wheelPivotControllers[index][2].value",
        "wheelSpinControllers[index][2].value",
    ):
        assert token in source

    assert "vrigLayeredControllers wheelGroup" not in source
    assert "((eulerAngles 0 0 steer) as quat)" in source
    assert "fn vrigWheelSpinRotation" in source
    assert "vrigWheelSpinRotation wheelSpin[index] forwardOffsetDeg" in source


def test_remove_rig_ungroups_generated_import_groups_before_deleting_helpers(
    source: str,
) -> None:
    cleanup = source[source.index("fn vrigRemoveGeneratedRig"):source.index("/* ------------------------------------------------------------------ *\n * Rig construction")]
    assert "vrigGeneratedGroupHeads" in cleanup
    assert "ungroup groupHead" in cleanup
    assert cleanup.index("ungroup groupHead") < cleanup.index("vrigRestoreOriginalState")
    assert 'deleteUserProp node "VRIG_bound"' in source
