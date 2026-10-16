from __future__ import annotations

import math
import sys
from dataclasses import replace
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server"))

from vehicle_dynamics import (  # noqa: E402
    VEHICLE_PRESETS,
    BodyPose,
    Intensity,
    ackermann_angles,
    body_response,
    fit_body_pose,
    max_centre_steer_deg,
    plan_speed_profile,
    steering_angle_deg,
    validate_profile,
)

G = 9.80665
CAR = VEHICLE_PRESETS["car"]


def _response(longitudinal=0.0, lateral=0.0, vertical=0.0, profile=CAR):
    return body_response(
        profile,
        longitudinal_mps2=longitudinal,
        lateral_mps2=lateral,
        vertical_mps2=vertical,
        intensity=Intensity(),
    )


def test_moderate_cornering_gives_a_moderate_lean():
    response = _response(lateral=0.5 * G)
    assert response.roll_deg == pytest.approx(-0.5 * CAR.roll_gradient_deg_per_g)
    assert abs(response.roll_deg) < CAR.max_roll_deg / 2.0


def test_squat_and_dive_follow_their_own_gradients():
    assert _response(longitudinal=0.3 * G).pitch_deg == pytest.approx(0.3 * CAR.squat_gradient_deg_per_g)
    assert _response(longitudinal=-0.5 * G).pitch_deg == pytest.approx(-0.5 * CAR.dive_gradient_deg_per_g)


@pytest.mark.parametrize("longitudinal, lateral", [(5.0, 0.0), (-8.0, 0.0), (0.0, 9.0), (4.0, -7.0)])
def test_flat_road_driving_never_sinks_the_body(longitudinal, lateral):
    # Weight transfer moves load between wheels; it does not add to it.
    assert _response(longitudinal=longitudinal, lateral=lateral).heave_m == 0.0


def test_a_dip_compresses_by_the_spring_deflection_for_that_load():
    omega = 2.0 * math.pi * CAR.ride_frequency_hz
    assert _response(vertical=3.0).heave_m == pytest.approx(-3.0 / omega**2)
    assert _response(vertical=-3.0).heave_m == pytest.approx(3.0 / omega**2)


def test_body_response_stays_within_every_limit():
    response = _response(longitudinal=50.0, lateral=50.0, vertical=200.0)
    assert abs(response.roll_deg) <= CAR.max_roll_deg
    assert abs(response.pitch_deg) <= CAR.max_pitch_deg
    assert abs(response.heave_m) <= CAR.spring_travel_m


@pytest.mark.parametrize(
    "field_name",
    (
        "roll_gradient_deg_per_g",
        "squat_gradient_deg_per_g",
        "dive_gradient_deg_per_g",
        "ride_frequency_hz",
        "steering_ratio",
    ),
)
def test_profile_rejects_non_positive_ride_values(field_name):
    with pytest.raises(ValueError, match=field_name):
        validate_profile(replace(CAR, **{field_name: 0.0}))


def test_profile_rejects_negative_understeer():
    with pytest.raises(ValueError, match="understeer"):
        validate_profile(replace(CAR, understeer_deg_per_g=-1.0))


def test_straight_line_needs_no_steering():
    assert steering_angle_deg(0.0, 30.0, CAR) == 0.0


def test_steering_follows_the_turn_direction():
    assert steering_angle_deg(0.05, 10.0, CAR) > 0.0
    assert steering_angle_deg(-0.05, 10.0, CAR) < 0.0


def test_a_faster_corner_needs_more_lock():
    slow = steering_angle_deg(0.05, 5.0, CAR)
    fast = steering_angle_deg(0.05, 15.0, CAR)
    assert fast > slow > math.degrees(math.atan(CAR.wheelbase * 0.05)) - 1e-9


def test_steering_never_exceeds_the_lock():
    assert steering_angle_deg(5.0, 40.0, CAR) == pytest.approx(CAR.max_steer_deg)


def test_generous_clearance_keeps_the_full_lock():
    assert max_centre_steer_deg(CAR, 90.0, 90.0) == pytest.approx(CAR.max_steer_deg)


def test_inner_wheel_clearance_limits_the_centre_angle():
    centre = max_centre_steer_deg(CAR, 20.0, 90.0)
    inner, outer = ackermann_angles(centre, wheelbase=CAR.wheelbase, track=CAR.track, max_steer_deg=90.0)
    assert inner == pytest.approx(20.0, abs=1e-6)
    assert outer < inner


def test_no_clearance_means_no_steering():
    assert max_centre_steer_deg(CAR, 0.0, 30.0) == 0.0


def _track(length=200.0, step=0.5, radius=25.0, corner_from=80.0, corner_to=120.0):
    distances = [index * step for index in range(int(length / step) + 1)]
    curvatures = [1.0 / radius if corner_from <= d <= corner_to else 0.0 for d in distances]
    return distances, curvatures


def test_drive_starts_and_ends_at_rest():
    distances, curvatures = _track()
    speeds = plan_speed_profile(distances, curvatures, CAR, target_speed_mps=30.0)
    assert speeds[0] == 0.0 and speeds[-1] == 0.0
    assert max(speeds) > 10.0


@pytest.mark.parametrize("grip", (0.35, 0.55, 1.0))
def test_cornering_stays_within_the_grip_used(grip):
    distances, curvatures = _track()
    speeds = plan_speed_profile(distances, curvatures, CAR, target_speed_mps=30.0, grip_usage=grip)
    limit = grip * CAR.lateral_grip_g * G
    assert all(v * v * abs(k) <= limit * (1.0 + 1e-9) for v, k in zip(speeds, curvatures))


def test_braking_and_cornering_share_the_grip():
    distances, curvatures = _track()
    grip = 0.55
    speeds = plan_speed_profile(distances, curvatures, CAR, target_speed_mps=30.0, grip_usage=grip)
    lateral_limit = grip * CAR.lateral_grip_g * G
    for index in range(len(speeds) - 1):
        span = distances[index + 1] - distances[index]
        longitudinal = (speeds[index + 1] ** 2 - speeds[index] ** 2) / (2.0 * span)
        if longitudinal < 0.0:
            lateral = speeds[index + 1] ** 2 * abs(curvatures[index + 1])
            usage = (lateral / lateral_limit) ** 2 + (longitudinal / (grip * CAR.braking_mps2)) ** 2
            assert usage <= 1.0 + 1e-6


def test_a_relaxed_driver_is_slower_everywhere():
    distances, curvatures = _track()
    relaxed = plan_speed_profile(distances, curvatures, CAR, target_speed_mps=30.0, grip_usage=0.35)
    limit = plan_speed_profile(distances, curvatures, CAR, target_speed_mps=30.0, grip_usage=1.0)
    assert all(slow <= fast + 1e-9 for slow, fast in zip(relaxed, limit))
    assert max(relaxed) < max(limit)


def test_acceleration_tapers_as_speed_rises():
    distances = [index * 1.0 for index in range(2001)]
    speeds = plan_speed_profile(distances, [0.0] * len(distances), CAR, target_speed_mps=CAR.top_speed_mps)

    def accel_at(index):
        return (speeds[index + 1] ** 2 - speeds[index] ** 2) / 2.0

    early = accel_at(5)
    late = accel_at(next(i for i, v in enumerate(speeds) if v > 0.6 * CAR.top_speed_mps))
    assert late < 0.8 * early


def test_speed_profile_rejects_invalid_grip_usage():
    distances, curvatures = _track()
    with pytest.raises(ValueError, match="grip_usage"):
        plan_speed_profile(distances, curvatures, CAR, target_speed_mps=30.0, grip_usage=1.5)


CORNERS = [(1.35, 0.72), (1.35, -0.72), (-1.35, 0.72), (-1.35, -0.72)]


def _rises(pose):
    return [
        pose.heave_m
        + forward * math.tan(math.radians(pose.pitch_deg))
        + left * math.tan(math.radians(pose.roll_deg))
        for forward, left in CORNERS
    ]


def test_a_pose_within_travel_is_untouched():
    pose = BodyPose(0.0, 0.5, 1.0)
    assert fit_body_pose(pose, CORNERS, [0.05] * 4, 0.08) == pose


def test_an_arch_that_would_touch_lifts_the_body_and_keeps_the_lean():
    pose = BodyPose(0.0, 0.0, 3.0)
    fitted = fit_body_pose(pose, CORNERS, [0.02] * 4, 0.08)
    assert fitted.roll_deg == pytest.approx(3.0)
    assert fitted.heave_m > 0.0
    assert min(_rises(fitted)) >= -0.02 - 1e-9


def test_a_lean_too_large_for_the_travel_is_reduced():
    pose = BodyPose(0.0, 0.0, 10.0)
    fitted = fit_body_pose(pose, CORNERS, [0.01] * 4, 0.02)
    assert 0.0 < fitted.roll_deg < 10.0
    rises = _rises(fitted)
    assert min(rises) >= -0.01 - 1e-9
    assert max(rises) <= 0.02 + 1e-9


def test_each_corner_keeps_its_own_compression_limit():
    pose = BodyPose(-0.03, -1.0, 0.0)
    limits = [0.01, 0.01, 0.04, 0.04]
    rises = _rises(fit_body_pose(pose, CORNERS, limits, 0.08))
    assert all(rise >= -limit - 1e-9 for rise, limit in zip(rises, limits))


def test_fit_body_pose_needs_a_limit_per_corner():
    with pytest.raises(ValueError, match="per corner"):
        fit_body_pose(BodyPose(0.0, 0.0, 0.0), CORNERS, [0.02], 0.08)
