from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server"))

from vehicle_dynamics import (  # noqa: E402
    SuspensionState,
    axle_pose,
    point_at_distance,
    suspension_step,
)


def _settle(target, seconds, dt, **kwargs):
    state = SuspensionState()
    history = []
    for _ in range(round(seconds / dt)):
        state = suspension_step(state, target, dt, **kwargs)
        history.append(state.value)
    return state, history


def test_suspension_settles_on_its_target():
    state, _ = _settle(1.0, 5.0, 1.0 / 30.0)
    assert state.value == pytest.approx(1.0, abs=1e-3)
    assert state.velocity == pytest.approx(0.0, abs=1e-2)


def test_underdamped_suspension_overshoots_only_a_little():
    _, history = _settle(1.0, 5.0, 1.0 / 30.0, damping_ratio=0.7)
    assert 1.0 < max(history) < 1.1


def test_critically_damped_suspension_never_overshoots():
    _, history = _settle(1.0, 5.0, 1.0 / 30.0, damping_ratio=1.0)
    assert max(history) <= 1.0 + 1e-9


def test_suspension_eases_into_a_step_instead_of_snapping():
    first = suspension_step(SuspensionState(), 1.0, 1.0 / 30.0)
    assert 0.0 < first.value < 0.2


def test_suspension_stays_stable_over_a_long_step():
    state = SuspensionState()
    for _ in range(20):
        state = suspension_step(state, 1.0, 0.5)
        assert abs(state.value) < 2.0
    assert state.value == pytest.approx(1.0, abs=1e-3)


def test_zero_step_leaves_the_suspension_unchanged():
    state = SuspensionState(value=0.3, velocity=-0.2)
    assert suspension_step(state, 1.0, 0.0) == state


@pytest.mark.parametrize(
    "kwargs",
    [
        {"dt": -0.1},
        {"dt": math.nan},
        {"dt": 0.1, "natural_hz": 0.0},
        {"dt": 0.1, "damping_ratio": 1.5},
    ],
)
def test_suspension_rejects_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        suspension_step(SuspensionState(), 1.0, **kwargs)


STRAIGHT = [(0.0, 0.0, 0.0), (100.0, 0.0, 0.0)]
CORNER = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 10.0, 0.0)]


def test_point_at_distance_interpolates_along_the_path():
    assert point_at_distance(CORNER, 15.0) == pytest.approx((10.0, 5.0, 0.0))


def test_point_at_distance_extrapolates_past_both_ends():
    assert point_at_distance(STRAIGHT, -5.0) == pytest.approx((-5.0, 0.0, 0.0))
    assert point_at_distance(CORNER, 25.0) == pytest.approx((10.0, 15.0, 0.0))


def test_point_at_distance_skips_a_duplicate_end_point():
    path = [(0.0, 0.0, 0.0), (10.0, 0.0, 0.0), (10.0, 0.0, 0.0)]
    assert point_at_distance(path, 12.0) == pytest.approx((12.0, 0.0, 0.0))


def test_straight_path_keeps_heading_and_level_pitch():
    pose = axle_pose(STRAIGHT, 50.0, wheelbase=2.7)
    assert pose.position == pytest.approx((50.0, 0.0, 0.0))
    assert pose.heading_deg == pytest.approx(0.0)
    assert pose.pitch_deg == pytest.approx(0.0)


def test_corner_knot_turns_the_vehicle_over_its_wheelbase():
    headings = [axle_pose(CORNER, s / 10.0, 2.7).heading_deg for s in range(50, 150)]
    steps = [abs(after - before) for before, after in zip(headings, headings[1:])]
    assert headings[0] == pytest.approx(0.0)
    assert headings[-1] == pytest.approx(90.0)
    assert max(steps) < 10.0


def test_vehicle_cuts_inside_a_corner_like_a_real_car():
    pose = axle_pose(CORNER, 10.0, 2.7)
    assert pose.position[0] < 10.0
    assert pose.position[1] > 0.0


def test_slope_pitches_the_nose_up():
    hill = [(0.0, 0.0, 0.0), (100.0, 0.0, 10.0)]
    pose = axle_pose(hill, 50.0, 2.7)
    assert pose.pitch_deg == pytest.approx(math.degrees(math.atan(0.1)))


def test_axle_pose_rejects_a_non_positive_wheelbase():
    with pytest.raises(ValueError, match="wheelbase"):
        axle_pose(STRAIGHT, 10.0, 0.0)
