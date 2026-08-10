from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server"))

from vehicle_dynamics import (  # noqa: E402
    Intensity,
    VEHICLE_PRESETS,
    VehicleProfile,
    ackermann_angles,
    body_response,
    curve_speed_limit,
    validate_profile,
    wheel_rotation_degrees,
)


def test_car_preset_has_safe_physical_dimensions():
    car = VEHICLE_PRESETS["car"]
    assert car.wheelbase > 2.0
    assert car.tyre_radius > 0.20
    assert car.max_steer_deg > 20.0


def test_inner_front_wheel_turns_further_than_outer_wheel():
    inner, outer = ackermann_angles(20.0, wheelbase=2.7, track=1.55)
    assert inner > outer > 0.0


def test_wheel_rotation_uses_distance_and_radius():
    assert wheel_rotation_degrees(2.0 * math.pi * 0.3, 0.3) == pytest.approx(360.0)


def test_invalid_profile_is_rejected():
    invalid = VehicleProfile(
        name="broken",
        wheelbase=0.0,
        track=1.5,
        tyre_radius=0.3,
        top_speed_mps=30.0,
        acceleration_mps2=4.0,
        braking_mps2=8.0,
        lateral_grip_g=0.9,
        max_steer_deg=30.0,
        spring_travel_m=0.1,
        damping_ratio=0.7,
        max_roll_deg=5.0,
        max_pitch_deg=3.0,
    )

    with pytest.raises(ValueError, match="wheelbase"):
        validate_profile(invalid)


@pytest.mark.parametrize(
    "field_name",
    (
        "wheelbase",
        "track",
        "tyre_radius",
        "top_speed_mps",
        "acceleration_mps2",
        "braking_mps2",
        "lateral_grip_g",
        "max_steer_deg",
        "spring_travel_m",
        "damping_ratio",
        "max_roll_deg",
        "max_pitch_deg",
    ),
)
@pytest.mark.parametrize("invalid_value", (math.nan, math.inf, -math.inf))
def test_profile_rejects_non_finite_physical_values(field_name, invalid_value):
    invalid = replace(VEHICLE_PRESETS["car"], **{field_name: invalid_value})

    with pytest.raises(ValueError, match=field_name):
        validate_profile(invalid)


def test_curve_speed_limit_respects_vehicle_top_speed():
    car = VEHICLE_PRESETS["car"]
    assert curve_speed_limit(1_000_000.0, car) == car.top_speed_mps


def test_ackermann_angles_are_clamped_to_steering_lock():
    inner, outer = ackermann_angles(80.0, wheelbase=2.7, track=1.55, max_steer_deg=25.0)
    assert 0.0 < outer <= inner <= 25.0


def test_ackermann_angles_preserve_negative_steering_sign():
    inner, outer = ackermann_angles(-20.0, wheelbase=2.7, track=1.55)
    assert inner < outer < 0.0


def test_wheel_rotation_preserves_negative_travel_sign():
    assert wheel_rotation_degrees(-2.0 * math.pi * 0.3, 0.3) == pytest.approx(-360.0)


def test_body_response_never_exceeds_profile_limits():
    profile = VEHICLE_PRESETS["suv"]
    response = body_response(
        profile,
        longitudinal_mps2=50.0,
        lateral_mps2=50.0,
        intensity=Intensity.all_max(),
    )
    assert abs(response.roll_deg) <= profile.max_roll_deg
    assert abs(response.pitch_deg) <= profile.max_pitch_deg
    assert abs(response.heave_m) <= profile.spring_travel_m
