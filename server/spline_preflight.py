"""Host-independent safety checks for an open vehicle path."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Literal, Sequence

from vehicle_dynamics import VehicleProfile, validate_profile


_EPSILON = 1e-9
_SHARP_KINK_RAD = math.radians(45.0)


@dataclass(frozen=True)
class SplineIssue:
    """A deterministic error or acknowledgement-required path warning."""

    code: str
    severity: Literal["warning", "error"]
    message: str
    index: int | None = None


def _distance_3d(start: Sequence[float], end: Sequence[float]) -> float:
    return math.dist(start, end)


def _orientation(
    start: Sequence[float], end: Sequence[float], point: Sequence[float]
) -> float:
    return (end[0] - start[0]) * (point[1] - start[1]) - (
        end[1] - start[1]
    ) * (point[0] - start[0])


def _point_on_segment(
    point: Sequence[float], start: Sequence[float], end: Sequence[float]
) -> bool:
    return (
        min(start[0], end[0]) - _EPSILON <= point[0] <= max(start[0], end[0]) + _EPSILON
        and min(start[1], end[1]) - _EPSILON
        <= point[1]
        <= max(start[1], end[1]) + _EPSILON
    )


def _segments_intersect_2d(
    first_start: Sequence[float],
    first_end: Sequence[float],
    second_start: Sequence[float],
    second_end: Sequence[float],
) -> bool:
    first_second_start = _orientation(first_start, first_end, second_start)
    first_second_end = _orientation(first_start, first_end, second_end)
    second_first_start = _orientation(second_start, second_end, first_start)
    second_first_end = _orientation(second_start, second_end, first_end)

    if (
        (first_second_start > _EPSILON and first_second_end < -_EPSILON)
        or (first_second_start < -_EPSILON and first_second_end > _EPSILON)
    ) and (
        (second_first_start > _EPSILON and second_first_end < -_EPSILON)
        or (second_first_start < -_EPSILON and second_first_end > _EPSILON)
    ):
        return True

    return (
        abs(first_second_start) <= _EPSILON
        and _point_on_segment(second_start, first_start, first_end)
    ) or (
        abs(first_second_end) <= _EPSILON
        and _point_on_segment(second_end, first_start, first_end)
    ) or (
        abs(second_first_start) <= _EPSILON
        and _point_on_segment(first_start, second_start, second_end)
    ) or (
        abs(second_first_end) <= _EPSILON
        and _point_on_segment(first_end, second_start, second_end)
    )


def _curvature_radius_2d(
    previous: Sequence[float], current: Sequence[float], following: Sequence[float]
) -> tuple[float | None, float]:
    incoming_x = current[0] - previous[0]
    incoming_y = current[1] - previous[1]
    outgoing_x = following[0] - current[0]
    outgoing_y = following[1] - current[1]
    incoming_length = math.hypot(incoming_x, incoming_y)
    outgoing_length = math.hypot(outgoing_x, outgoing_y)
    if incoming_length <= _EPSILON or outgoing_length <= _EPSILON:
        return None, 0.0

    cosine = max(
        -1.0,
        min(1.0, (incoming_x * outgoing_x + incoming_y * outgoing_y) / (incoming_length * outgoing_length)),
    )
    turn_angle = math.acos(cosine)
    chord_length = math.hypot(following[0] - previous[0], following[1] - previous[1])
    twice_area = abs(incoming_x * outgoing_y - incoming_y * outgoing_x)
    if twice_area <= _EPSILON:
        return None, turn_angle
    return incoming_length * outgoing_length * chord_length / (2.0 * twice_area), turn_angle


def preflight_open_path(
    points: Sequence[tuple[float, float, float]], profile: VehicleProfile
) -> list[SplineIssue]:
    """Return errors and acknowledgement-required warnings for an open path."""

    validate_profile(profile)
    path = list(points)
    issues: list[SplineIssue] = []

    if len({tuple(point) for point in path}) < 2:
        issues.append(
            SplineIssue("too_short", "error", "Path needs at least two distinct points")
        )

    zero_length_segments: set[int] = set()
    for index, (start, end) in enumerate(zip(path, path[1:])):
        if _distance_3d(start, end) <= _EPSILON:
            zero_length_segments.add(index)
            issues.append(
                SplineIssue(
                    "zero_length_segment",
                    "error",
                    "Path contains a zero-length segment",
                    index,
                )
            )
            issues.append(
                SplineIssue(
                    "duplicate_knot",
                    "error",
                    "Adjacent path points are duplicates",
                    index + 1,
                )
            )

    for first_index, (first_start, first_end) in enumerate(zip(path, path[1:])):
        if first_index in zero_length_segments:
            continue
        for second_index in range(first_index + 2, len(path) - 1):
            if second_index in zero_length_segments:
                continue
            if _segments_intersect_2d(
                first_start,
                first_end,
                path[second_index],
                path[second_index + 1],
            ):
                issues.append(
                    SplineIssue(
                        "self_intersection",
                        "warning",
                        "Path crosses itself in the vehicle travel plane",
                        second_index,
                    )
                )

    for index in range(1, len(path) - 1):
        radius, turn_angle = _curvature_radius_2d(
            path[index - 1], path[index], path[index + 1]
        )
        if turn_angle >= _SHARP_KINK_RAD:
            issues.append(
                SplineIssue(
                    "sharp_kink",
                    "warning",
                    "Path turns abruptly; smooth this knot before baking",
                    index,
                )
            )
        if radius is not None:
            required_steer_deg = math.degrees(math.atan(profile.wheelbase / radius))
            if required_steer_deg > profile.max_steer_deg:
                issues.append(
                    SplineIssue(
                        "steering_limit",
                        "warning",
                        "Path curvature exceeds the vehicle steering limit",
                        index,
                    )
                )

    return issues
