"""Deterministic, host-independent vehicle motion calculations in SI units."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping


_GRAVITY_MPS2 = 9.80665


@dataclass(frozen=True)
class VehicleProfile:
    """Physical limits used to plan and bake a vehicle animation."""

    name: str
    wheelbase: float
    track: float
    tyre_radius: float
    top_speed_mps: float
    acceleration_mps2: float
    braking_mps2: float
    lateral_grip_g: float
    max_steer_deg: float
    spring_travel_m: float
    damping_ratio: float
    max_roll_deg: float
    max_pitch_deg: float


@dataclass(frozen=True)
class Intensity:
    """Normalised artistic multipliers for the generated body motion."""

    acceleration: float = 1.0
    braking: float = 1.0
    body_roll: float = 1.0
    suspension: float = 1.0

    @classmethod
    def all_max(cls) -> "Intensity":
        return cls()


@dataclass(frozen=True)
class BodyResponse:
    """Bounded chassis attitude relative to the vehicle's resting pose."""

    roll_deg: float
    pitch_deg: float
    heave_m: float


def _profile(
    name: str,
    wheelbase: float,
    track: float,
    tyre_radius: float,
    top_speed_mps: float,
    acceleration_mps2: float,
    braking_mps2: float,
    lateral_grip_g: float,
    max_steer_deg: float,
    spring_travel_m: float,
    damping_ratio: float,
    max_roll_deg: float,
    max_pitch_deg: float,
) -> VehicleProfile:
    profile = VehicleProfile(
        name=name,
        wheelbase=wheelbase,
        track=track,
        tyre_radius=tyre_radius,
        top_speed_mps=top_speed_mps,
        acceleration_mps2=acceleration_mps2,
        braking_mps2=braking_mps2,
        lateral_grip_g=lateral_grip_g,
        max_steer_deg=max_steer_deg,
        spring_travel_m=spring_travel_m,
        damping_ratio=damping_ratio,
        max_roll_deg=max_roll_deg,
        max_pitch_deg=max_pitch_deg,
    )
    return profile


VEHICLE_PRESETS: Mapping[str, VehicleProfile] = {
    "car": _profile("car", 2.70, 1.55, 0.31, 70.0, 5.5, 9.0, 1.00, 35.0, 0.12, 0.70, 6.0, 4.0),
    "suv": _profile("suv", 2.85, 1.65, 0.36, 55.0, 4.0, 8.0, 0.85, 32.0, 0.18, 0.75, 9.0, 5.0),
    "pickup": _profile("pickup", 3.20, 1.72, 0.38, 48.0, 3.2, 7.0, 0.75, 30.0, 0.20, 0.80, 10.0, 5.5),
    "van": _profile("van", 3.10, 1.70, 0.35, 42.0, 3.0, 6.5, 0.70, 28.0, 0.18, 0.80, 10.0, 5.0),
    "bus": _profile("bus", 6.00, 2.35, 0.49, 30.0, 1.5, 5.0, 0.55, 24.0, 0.16, 0.85, 7.0, 3.5),
}


def validate_profile(profile: VehicleProfile) -> None:
    """Raise ``ValueError`` when a profile cannot represent a safe vehicle."""

    if not isinstance(profile, VehicleProfile):
        raise TypeError("profile must be a VehicleProfile")
    if not profile.name.strip():
        raise ValueError("profile name must not be empty")
    positive_fields = (
        "wheelbase",
        "track",
        "tyre_radius",
        "top_speed_mps",
        "acceleration_mps2",
        "braking_mps2",
        "lateral_grip_g",
        "max_steer_deg",
        "spring_travel_m",
        "max_roll_deg",
        "max_pitch_deg",
    )
    for field_name in positive_fields:
        if getattr(profile, field_name) <= 0.0:
            raise ValueError(f"{field_name} must be greater than zero")
    if not 0.0 <= profile.damping_ratio <= 1.0:
        raise ValueError("damping_ratio must be between zero and one")


def curve_speed_limit(radius_m: float, profile: VehicleProfile) -> float:
    """Return the grip-limited speed on a constant-radius curve, in m/s."""

    validate_profile(profile)
    return min(
        profile.top_speed_mps,
        math.sqrt(max(radius_m, 0.0) * _GRAVITY_MPS2 * profile.lateral_grip_g),
    )


def ackermann_angles(
    steering_deg: float,
    *,
    wheelbase: float,
    track: float,
    max_steer_deg: float = 45.0,
) -> tuple[float, float]:
    """Return signed inner and outer front-wheel steering angles in degrees."""

    if wheelbase <= 0.0:
        raise ValueError("wheelbase must be greater than zero")
    if track <= 0.0:
        raise ValueError("track must be greater than zero")
    if max_steer_deg <= 0.0:
        raise ValueError("max_steer_deg must be greater than zero")
    if steering_deg == 0.0:
        return (0.0, 0.0)

    sign = math.copysign(1.0, steering_deg)
    centre_angle_rad = math.radians(min(abs(steering_deg), max_steer_deg))
    turn_radius = wheelbase / math.tan(centre_angle_rad)
    inner_radius = max(turn_radius - track / 2.0, 0.0)
    outer_radius = turn_radius + track / 2.0
    inner = math.degrees(math.atan(wheelbase / inner_radius)) if inner_radius else 90.0
    outer = math.degrees(math.atan(wheelbase / outer_radius))
    return (
        sign * min(inner, max_steer_deg),
        sign * min(outer, max_steer_deg),
    )


def wheel_rotation_degrees(distance_m: float, tyre_radius_m: float) -> float:
    """Return signed tyre rotation for travelled distance, in degrees."""

    if tyre_radius_m <= 0.0:
        raise ValueError("tyre_radius_m must be greater than zero")
    return math.degrees(distance_m / tyre_radius_m)


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(value, limit))


def _validate_intensity(intensity: Intensity) -> None:
    if not isinstance(intensity, Intensity):
        raise TypeError("intensity must be an Intensity")
    for field_name in ("acceleration", "braking", "body_roll", "suspension"):
        value = getattr(intensity, field_name)
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"intensity.{field_name} must be between zero and one")


def body_response(
    profile: VehicleProfile,
    *,
    longitudinal_mps2: float,
    lateral_mps2: float,
    intensity: Intensity,
) -> BodyResponse:
    """Calculate a stable, bounded chassis response from linear accelerations."""

    validate_profile(profile)
    _validate_intensity(intensity)
    lateral_limit = _GRAVITY_MPS2 * profile.lateral_grip_g
    roll = _clamp(
        -lateral_mps2 / lateral_limit * profile.max_roll_deg * intensity.body_roll,
        profile.max_roll_deg,
    )
    longitudinal_limit = (
        profile.acceleration_mps2 if longitudinal_mps2 >= 0.0 else profile.braking_mps2
    )
    longitudinal_intensity = (
        intensity.acceleration if longitudinal_mps2 >= 0.0 else intensity.braking
    )
    pitch = _clamp(
        longitudinal_mps2 / longitudinal_limit * profile.max_pitch_deg * longitudinal_intensity,
        profile.max_pitch_deg,
    )
    load_ratio = min(
        1.0,
        math.hypot(
            longitudinal_mps2 / longitudinal_limit,
            lateral_mps2 / lateral_limit,
        ),
    )
    heave = -load_ratio * profile.spring_travel_m * intensity.suspension
    return BodyResponse(roll_deg=roll, pitch_deg=pitch, heave_m=heave)
