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
    "rollGradient": "roll_gradient_deg_per_g",
    "squatGradient": "squat_gradient_deg_per_g",
    "diveGradient": "dive_gradient_deg_per_g",
    "rideFrequency": "ride_frequency_hz",
    "understeer": "understeer_deg_per_g",
    "steeringRatio": "steering_ratio",
}

_KEYWORD = re.compile(r'(\w+):(?:"([^"]*)"|(-?\d+\.?\d*))')


@pytest.fixture(scope="module")
def source() -> str:
    return SCRIPT.read_text(encoding="utf-8")


@pytest.fixture(scope="module")
def code(source: str) -> str:
    """The script with comments and string literals blanked out."""

    without_blocks = re.sub(r"/\*.*?\*/", "", source, flags=re.S)
    without_lines = re.sub(r"--[^\n]*", "", without_blocks)
    return re.sub(r'"[^"\n]*"', '""', without_lines)


def _function(source: str, name: str) -> str:
    start = source.index(f"fn {name} ")
    following = re.search(r"\n(fn |rollout )", source[start + 1:])
    return source[start:start + 1 + following.start()] if following else source[start:]


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


def _axis_table(source: str, name: str) -> dict[int, tuple[float, float, float]]:
    body = _function(source, name)
    table = {}
    for index, component, sign in re.findall(r"(\d):\s*#\((\d),\s*(-?\d+\.\d+)\)", body):
        vector = [0.0, 0.0, 0.0]
        vector[int(component) - 1] = float(sign)
        table[int(index)] = tuple(vector)
    return table


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
    declarations += re.findall(r"\bfor\s+([A-Za-z_]\w*)\s+(?:in|=)", source)
    clashes = sorted({name for name in declarations if name in MAXSCRIPT_KEYWORDS})
    assert not clashes, f"reserved MaxScript words used as identifiers: {clashes}"


def test_no_function_is_used_before_it_is_defined(code: str) -> None:
    # Max binds names when it compiles a function. A call to a function that
    # is defined further down the file compiles to an undefined variable and
    # fails at run time with "Call needs function or class, got: undefined".
    early = []
    for definition in re.finditer(r"\bfn\s+(\w+)", code):
        name = definition.group(1)
        if re.search(rf"\b{name}\b", code).start() < definition.start(1):
            early.append(name)
    assert not early, f"used before definition: {early}"


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


def test_preflight_reports_knot_errors_by_knot_number(source: str) -> None:
    preflight = _function(source, "vrigPreflightPath")
    assert "vrigPreflightOpenPath knotPoints profile" in preflight
    assert '" at knot "' in _function(source, "vrigFormatIssues")


def test_spline_sampler_follows_the_drawn_curve(source: str, code: str) -> None:
    # lengthInterp returns repeated, polyline-snapped points on 3ds Max 2027;
    # the sampler evaluates each bezier segment from its knots and handles.
    sampler = _function(source, "vrigSamplePath")
    for token in ("getKnotPoint", "getOutVec", "getInVec", "vrigBezierPoint"):
        assert token in sampler
    assert "lengthInterp" not in code


def test_heading_comes_from_both_axles_standing_on_the_path(source: str) -> None:
    pose = _function(source, "vrigAxlePose")
    assert "travelled - wheelbase / 2.0" in pose
    assert "travelled + wheelbase / 2.0" in pose
    bake = _function(source, "vrigBakeAnimation")
    assert "vrigAxlePose path travelled profile.wheelbase" in bake
    # Integrated and clamped yaw drifted away from the path at corner knots.
    assert "vrigYawDeltaDegrees" not in source
    assert "maxYawStep" not in source


def test_rotations_are_written_to_euler_tracks_not_quaternions(source: str) -> None:
    # A quaternion written per frame wraps at 180 degrees, so a spinning
    # wheel or a heading through south flips backwards between keys.
    bake = _function(source, "vrigBakeAnimation")
    assert "as quat" not in bake
    assert "vrigSetEulerAxis" in bake
    setter = _function(source, "vrigSetEulerAxis")
    for track in ("x_rotation", "y_rotation", "z_rotation"):
        assert track in setter
    assert "vrigUnwrapDegrees yaw previousYaw" in bake


def test_vehicle_axes_follow_the_measured_facing(source: str) -> None:
    forward = _axis_table(source, "vrigForwardAxis")
    left = _axis_table(source, "vrigLeftAxis")
    expected_forward = {
        1: (1.0, 0.0, 0.0),
        2: (-1.0, 0.0, 0.0),
        3: (0.0, 1.0, 0.0),
        4: (0.0, -1.0, 0.0),
    }
    assert forward == expected_forward
    for index, (fx, fy, fz) in expected_forward.items():
        # left = up x forward; positive rotation about it rolls a wheel forward.
        up_cross_forward = (0.0 * fz - 1.0 * fy, 1.0 * fx - 0.0 * fz, 0.0)
        assert left[index] == pytest.approx(up_cross_forward)


def test_wheels_spin_about_the_left_axis_and_steer_about_up(source: str) -> None:
    bake = _function(source, "vrigBakeAnimation")
    assert "vrigSetEulerAxis spinControllers[index][2] leftAxis[1] (leftAxis[2] * wheelSpin[index])" in bake
    assert "vrigSetEulerAxis pivotControllers[1][2] 3 steerLeft" in bake
    assert "vrigSetEulerAxis pivotControllers[2][2] 3 steerRight" in bake
    assert "rolled / profile.tyreRadius" in bake


def test_body_rolls_outward_and_pitches_nose_up_under_power(source: str) -> None:
    bake = _function(source, "vrigBakeAnimation")
    # The fitted pose carries left-side-up roll, so a left turn (negative
    # body_response roll) lifts the left side: the body leans outward.
    assert "vrigFitBodyPose target[3] target[2] (-target[1])" in bake
    assert "forwardAxis[1] (forwardAxis[2] * bodyPose[3])" in bake
    assert "leftAxis[1] (-1.0 * leftAxis[2] * bodyPose[2])" in bake
    assert "vrigSuspensionStep" in bake


def test_body_is_fitted_inside_the_arches_before_and_after_the_springs(source: str) -> None:
    bake = _function(source, "vrigBakeAnimation")
    before = bake.index("vrigFitBodyPose target[3]")
    spring = bake.index("vrigSuspensionStep heaveState")
    after = bake.index("vrigFitBodyPose heaveState[1]")
    assert before < spring < after


def test_flat_road_driving_never_sinks_the_body(source: str) -> None:
    response = _function(source, "vrigBodyResponse")
    assert "loadRatio" not in source
    assert "-vertical / (omega * omega)" in response
    assert "profile.rollGradient" in response
    assert "profile.squatGradient" in response and "profile.diveGradient" in response


def test_body_fit_lifts_before_it_reduces_the_lean(source: str) -> None:
    fit = _function(source, "vrigFitBodyPose")
    assert "amin (amax heave window[1]) window[2]" in fit
    assert "vrigHeightWindow" in fit


def test_clearance_comes_from_the_models_own_arches(source: str) -> None:
    measure = _function(source, "vrigMeasureClearance")
    # Liners modelled inside the tyre at rest would forbid all motion.
    assert "appendIfUnique ignored" in measure
    assert "intersectRay tyrePoint [0, 0, 1] true" in measure
    assert "vrigSteerClearance active points centre 1.0 compression[index]" in measure
    # A grid keeps only the first node added, so each mesh gets its own.
    grids = _function(source, "vrigMeshGrids")
    assert "for node in nodes collect" in grids and "grid.addNode node" in grids
    bake = _function(source, "vrigBakeAnimation")
    assert "vrigCachedClearance rootNode clearanceBody wheelNodes measure profile scale" in bake


def test_bake_eases_the_body_off_the_real_arches(source: str) -> None:
    # Travel limits are measured straight up, but a rolling body also slides
    # its arches sideways; near the limits each frame is checked against the
    # model's own meshes and the body eased back only as far as needed.
    bake = _function(source, "vrigBakeAnimation")
    assert "vrigPoseTouches clearance (vrigPoseMatrix bodyPose 1.0" in bake
    assert "bodyPose = #(bodyPose[1] * low, bodyPose[2] * low, bodyPose[3] * low)" in bake
    assert "easedFrames" in bake and "unresolvedFrames" in bake


def test_contact_check_moves_the_body_exactly_as_the_bake_does(source: str) -> None:
    pose = _function(source, "vrigPoseMatrix")
    assert "values[forwardAxis[1]] = forwardAxis[2] * pose[3] * share" in pose
    assert "values[leftAxis[1]] = -1.0 * leftAxis[2] * pose[2] * share" in pose
    bake = _function(source, "vrigBakeAnimation")
    assert "forwardAxis[1] (forwardAxis[2] * bodyPose[3])" in bake
    assert "leftAxis[1] (-1.0 * leftAxis[2] * bodyPose[2])" in bake
    matrix = _function(source, "vrigBodyMatrix")
    assert "(eulerAngles eulerValues.x eulerValues.y eulerValues.z) as matrix3" in matrix


def test_contact_check_keeps_a_safety_skin_and_prunes_far_meshes(source: str) -> None:
    measure = _function(source, "vrigMeasureClearance")
    assert "(radius + 0.003 * scale) (halfWidth + 0.002 * scale)" in measure
    assert "vrigGridWithin active[slot] points centre within" in measure


def test_clearance_cache_holds_one_vehicle_and_frees_its_grids(source: str) -> None:
    cached = _function(source, "vrigCachedClearance")
    assert "vrigFreeGrids entry[2].grids" in cached
    assert "vrigClearanceCache = #(#(key, found))" in cached
    assert "(measure.groundCentre as string)" in _function(source, "vrigClearanceKey")


def test_steering_stops_at_the_lock_the_arches_allow(source: str) -> None:
    bake = _function(source, "vrigBakeAnimation")
    assert "local steerLimits = vrigSteerLimits clearance profile" in bake
    assert "amin wantedSteer steerLimits[1]" in bake
    assert "amax wantedSteer (-steerLimits[2])" in bake
    assert "vrigMaxCentreSteer profile clearance.leftTurnLimits[1]" in source


def test_driving_shares_grip_between_cornering_and_braking(source: str) -> None:
    speeds = _function(source, "vrigSpeedProfile")
    assert "vrigLongitudinalAvailable drive speed" in speeds
    assert "vrigLongitudinalAvailable (gripUsage * profile.braking)" in speeds
    assert "(1.0 - (vrigSquare (speed / profile.topSpeed)))" in speeds
    styles = _function(source, "vrigDrivingStyles")
    for style in ('"Relaxed", 0.35', '"Normal", 0.55', '"Sporty", 0.75', '"Limit", 1.0'):
        assert style in styles
    assert "Driving style" in source


def test_the_report_wraps_long_lines_to_fit_the_box(source: str) -> None:
    rollout = source[source.index("rollout VehicleRigRollout"):]
    assert "for row in (vrigWrapRows line 58) do text += row" in rollout
    wrap = _function(source, "vrigWrapRows")
    assert "filterString sentence \" \"" in wrap
    assert "(current.count + 1 + word.count) > columns" in wrap


def test_each_span_is_crossed_at_constant_acceleration(source: str) -> None:
    state = _function(source, "vrigStateAtTime")
    assert "0.5 * accel * tau * tau" in state
    assert "motion.speeds[low] + accel * tau" in state
    assert "(speeds[index] + speeds[index + 1]) / 2.0" in _function(source, "vrigTravelTimes")


def test_the_body_settles_after_the_vehicle_stops(source: str) -> None:
    bake = _function(source, "vrigBakeAnimation")
    assert "motion.duration + settings.settleSeconds" in bake


def test_steering_wheel_turns_about_its_own_column(source: str) -> None:
    assert '"steeringwheel"' in _function(source, "vrigIsSteeringWheelPart")
    column = _function(source, "vrigCreateSteeringColumn")
    assert "vrigAxisMatrix axisInfo[2] axisInfo[1]" in column
    assert "column.parent = body" in column
    bake = _function(source, "vrigBakeAnimation")
    assert "vrigSetEulerAxis steeringControllers[2] 3 (steerCentre * profile.steeringRatio)" in bake


def test_test_roads_use_constant_radius_bends(source: str) -> None:
    knots = _function(source, "vrigRoadKnots")
    assert "4.0 / 3.0 * (tan ((abs sweep) / 4.0)) * radius" in knots
    assert "#bezierCorner" in _function(source, "vrigBuildRoad")


def test_suspension_mirrors_the_python_spring_damper(source: str) -> None:
    step = _function(source, "vrigSuspensionStep")
    assert "ceil (dt * omega * 4.0)" in step
    assert "omega * omega * (target - value) - 2.0 * dampingRatio * omega * velocity" in step


def test_vehicle_is_measured_from_evaluated_geometry(source: str) -> None:
    measure = _function(source, "vrigMeasureVehicle")
    assert "vrigGeometryUnder wheelNodes[index]" in measure
    assert "vrigWorldBounds" in measure
    assert "(bounds[2].z - bounds[1].z) / 2.0" in measure
    assert "nodeGetBoundingBox node (matrix3 1)" in _function(source, "vrigWorldBounds")


def test_helpers_sit_at_measured_points_with_world_axes(source: str) -> None:
    helpers = _function(source, "vrigCreateHelpers")
    assert "transMatrix measure.wheelCentres[index]" in helpers
    assert "measure.groundCentre" in helpers
    # Copying a source node's transform placed pivots wherever the import's
    # object pivot happened to be.
    assert ".transform = wheelNodes" not in source
    assert ".transform = bodyNode" not in source


def test_scale_is_fitted_only_when_scene_units_are_implausible(source: str) -> None:
    fit = _function(source, "vrigSuggestedScale")
    assert "wheelbaseMetres >= 1.5 and wheelbaseMetres <= 10.0" in fit
    assert "measure.wheelbase / presetWheelbase" in fit


def test_bake_uses_measured_facing_unless_overridden(source: str) -> None:
    bake = _function(source, "vrigBakeAnimation")
    assert "if settings.forwardIndex == undefined then measure.forwardIndex" in bake
    assert "forwardOverride" in source


def test_frame_range_can_match_the_drive_time(source: str) -> None:
    assert "Match frame range to drive time" in source
    assert "if settings.matchDriveTime do" in _function(source, "vrigBakeAnimation")


def test_bake_sets_the_requested_animation_range(source: str) -> None:
    assert "animationRange = interval startFrame endFrame" in source


def test_source_nodes_are_bound_before_helper_keys_are_baked(source: str) -> None:
    bake = _function(source, "vrigBakeAnimation")
    assert bake.index("rootNode.parent = master") < bake.index("with animate on")


def test_a_second_bake_replaces_the_first(source: str) -> None:
    bake = _function(source, "vrigBakeAnimation")
    assert bake.index("vrigUnbake()") < bake.index("vrigMeasureVehicle")


def test_bake_and_assembly_keep_separate_restore_records(source: str) -> None:
    keys = _function(source, "vrigRecordKeys")
    assert '"VRIG_baked"' in keys and '"VRIG_bound"' in keys
    record = _function(source, "vrigRecordOriginalState")
    # A record is written once; re-binding must not replace the original.
    assert "if (getUserProp node keys[1]) != undefined then false" in record
    assert 'vrigRestoreOriginalStates "bake"' in _function(source, "vrigUnbake")


def test_restore_relinks_parents_by_handle(source: str) -> None:
    assert "originalParent.inode.handle" in _function(source, "vrigRecordOriginalState")
    assert "maxOps.getNodeByHandle handle" in _function(source, "vrigRecordedParent")


def test_restore_unlinks_every_node_before_restoring_transforms(source: str) -> None:
    restore = _function(source, "vrigRestoreOriginalStates")
    unlink = restore.index("for node in records do node.parent = undefined")
    assert unlink < restore.index(".transform = restored")
    assert "deleteUserProp node key" in restore


def test_remove_rig_ungroups_generated_import_groups_before_restoring(source: str) -> None:
    cleanup = _function(source, "vrigRemoveGeneratedRig")
    assert "vrigGeneratedGroupHeads" in cleanup
    assert "ungroup groupHead" in cleanup
    assert cleanup.index("vrigUnbake()") < cleanup.index("ungroup groupHead")
    assert cleanup.index("ungroup groupHead") < cleanup.index('vrigRestoreOriginalStates "assembly"')


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


def test_imported_vehicle_assembly_uses_native_groups(source: str) -> None:
    assembly = _function(source, "vrigAssembleSelectedVehicle")
    for token in (
        "group bodySet name:",
        "_BodyGroup",
        "_WheelGroup_",
        "bodyGroup.parent = bodyCarrier",
        "wheelGroup.parent = wheel",
    ):
        assert token in assembly


def test_brake_parts_steer_without_spinning(source: str) -> None:
    brake = _function(source, "vrigIsBrakePart")
    assert '"caliper"' in brake and '"brake"' in brake
    assert "brakeNodes[index].parent = wheelPivots[index]" in _function(source, "vrigBakeAnimation")


def test_assembly_refuses_an_imported_parent_shared_between_clusters(source: str) -> None:
    assembly = _function(source, "vrigAssembleSelectedVehicle")
    assert "Detach or ungroup them before assembly." in assembly
    # Every failure after mutation rolls the scene back.
    assert assembly.count("vrigRemoveGeneratedRig()") >= 3


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


def test_path_fixtures_are_laid_out_in_metres_from_the_vehicle(source: str) -> None:
    fixtures = _function(source, "vrigCreateTestPaths")
    assert "fn vrigCreateTestPaths origin forwardIndex scale" in fixtures
    assert "* scale" in fixtures
