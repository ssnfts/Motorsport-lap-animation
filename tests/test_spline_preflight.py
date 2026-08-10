from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "server"))

from spline_preflight import SplineIssue, preflight_open_path  # noqa: E402
from vehicle_dynamics import VEHICLE_PRESETS  # noqa: E402


def _has_issue(issues, code, severity):
    return any(issue.code == code and issue.severity == severity for issue in issues)


def test_insufficient_distinct_points_is_an_error():
    issues = preflight_open_path([(0, 0, 0), (0, 0, 0)], VEHICLE_PRESETS["car"])

    assert _has_issue(issues, "too_short", "error")


def test_duplicate_knot_is_an_error():
    issues = preflight_open_path(
        [(0, 0, 0), (1, 0, 0), (1, 0, 0)], VEHICLE_PRESETS["car"]
    )

    assert _has_issue(issues, "duplicate_knot", "error")
    assert _has_issue(issues, "zero_length_segment", "error")


def test_sharp_kink_is_a_warning():
    issues = preflight_open_path(
        [(0, 0, 0), (1, 0, 0), (1, 1, 0)], VEHICLE_PRESETS["car"]
    )

    assert _has_issue(issues, "sharp_kink", "warning")


def test_self_intersection_is_a_warning():
    issues = preflight_open_path(
        [(0, 0, 0), (2, 2, 0), (0, 2, 0), (2, 0, 0)], VEHICLE_PRESETS["car"]
    )

    assert _has_issue(issues, "self_intersection", "warning")


def test_tight_curve_is_reported_before_baking():
    issues = preflight_open_path(
        [(0, 0, 0), (1, 0, 0), (1, 0.1, 0)], VEHICLE_PRESETS["bus"]
    )

    assert _has_issue(issues, "steering_limit", "warning")


def test_straight_open_path_is_clear():
    issues = preflight_open_path(
        [(0, 0, 0), (5, 0, 0), (10, 0, 0)], VEHICLE_PRESETS["car"]
    )

    assert issues == []


def test_issue_is_an_immutable_value_object():
    issue = SplineIssue("sharp_kink", "warning", "Path turns abruptly", 1)

    assert issue.index == 1
