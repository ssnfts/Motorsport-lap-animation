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
    # How far the body leans per g of lateral or longitudinal acceleration.
    # Real road cars sit around 3-5 deg/g of roll and 1-3 deg/g of pitch;
    # the max_* limits above only cap these.
    roll_gradient_deg_per_g: float = 3.5
    squat_gradient_deg_per_g: float = 1.5
    dive_gradient_deg_per_g: float = 2.0
    # Body natural frequency on its springs. It sets how fast the body
    # settles and, through a / (2 pi f)^2, how far it sinks per g of load.
    ride_frequency_hz: float = 1.4
    # Extra front-wheel angle per g of cornering on top of the geometric
    # angle, which is what makes a car at speed need more lock.
    understeer_deg_per_g: float = 1.5
    # Steering-wheel turns per road-wheel turn.
    steering_ratio: float = 15.0


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
    **ride: float,
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
        **ride,
    )
    return profile


VEHICLE_PRESETS: Mapping[str, VehicleProfile] = {
    "car": _profile(
        "car", 2.70, 1.55, 0.31, 70.0, 5.5, 9.0, 1.00, 35.0, 0.12, 0.70, 6.0, 4.0,
        roll_gradient_deg_per_g=3.5, squat_gradient_deg_per_g=1.5, dive_gradient_deg_per_g=2.0,
        ride_frequency_hz=1.4, understeer_deg_per_g=1.5, steering_ratio=15.0,
    ),
    "suv": _profile(
        "suv", 2.85, 1.65, 0.36, 55.0, 4.0, 8.0, 0.85, 32.0, 0.18, 0.75, 9.0, 5.0,
        roll_gradient_deg_per_g=5.5, squat_gradient_deg_per_g=2.0, dive_gradient_deg_per_g=2.5,
        ride_frequency_hz=1.2, understeer_deg_per_g=2.5, steering_ratio=16.0,
    ),
    "pickup": _profile(
        "pickup", 3.20, 1.72, 0.38, 48.0, 3.2, 7.0, 0.75, 30.0, 0.20, 0.80, 10.0, 5.5,
        roll_gradient_deg_per_g=6.0, squat_gradient_deg_per_g=2.5, dive_gradient_deg_per_g=3.0,
        ride_frequency_hz=1.2, understeer_deg_per_g=3.0, steering_ratio=18.0,
    ),
    "van": _profile(
        "van", 3.10, 1.70, 0.35, 42.0, 3.0, 6.5, 0.70, 28.0, 0.18, 0.80, 10.0, 5.0,
        roll_gradient_deg_per_g=6.0, squat_gradient_deg_per_g=2.0, dive_gradient_deg_per_g=2.5,
        ride_frequency_hz=1.2, understeer_deg_per_g=3.0, steering_ratio=17.0,
    ),
    "bus": _profile(
        "bus", 6.00, 2.35, 0.49, 30.0, 1.5, 5.0, 0.55, 24.0, 0.16, 0.85, 7.0, 3.5,
        roll_gradient_deg_per_g=7.0, squat_gradient_deg_per_g=1.5, dive_gradient_deg_per_g=2.0,
        ride_frequency_hz=1.0, understeer_deg_per_g=3.5, steering_ratio=20.0,
    ),
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
        "roll_gradient_deg_per_g",
        "squat_gradient_deg_per_g",
        "dive_gradient_deg_per_g",
        "ride_frequency_hz",
        "steering_ratio",
    )
    for field_name in (*positive_fields, "damping_ratio", "understeer_deg_per_g"):
        if not math.isfinite(getattr(profile, field_name)):
            raise ValueError(f"{field_name} must be finite")
    for field_name in positive_fields:
        if getattr(profile, field_name) <= 0.0:
            raise ValueError(f"{field_name} must be greater than zero")
    if not 0.0 <= profile.damping_ratio <= 1.0:
        raise ValueError("damping_ratio must be between zero and one")
    if profile.understeer_deg_per_g < 0.0:
        raise ValueError("understeer_deg_per_g must not be negative")


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
    vertical_mps2: float = 0.0,
) -> BodyResponse:
    """The pose the chassis settles to under steady accelerations.

    Roll and pitch follow the profile's gradients, so moderate driving gives
    a moderate lean and only the limits cap it. Weight transfer redistributes
    load between the wheels rather than adding to it, so accelerating or
    cornering on a flat road does not sink the body: heave comes only from
    ``vertical_mps2``, the road's upward acceleration through a dip
    (positive) or over a crest (negative).
    """

    validate_profile(profile)
    _validate_intensity(intensity)
    roll = _clamp(
        -lateral_mps2 / _GRAVITY_MPS2 * profile.roll_gradient_deg_per_g * intensity.body_roll,
        profile.max_roll_deg,
    )
    if longitudinal_mps2 >= 0.0:
        gradient, scale = profile.squat_gradient_deg_per_g, intensity.acceleration
    else:
        gradient, scale = profile.dive_gradient_deg_per_g, intensity.braking
    pitch = _clamp(longitudinal_mps2 / _GRAVITY_MPS2 * gradient * scale, profile.max_pitch_deg)
    omega = 2.0 * math.pi * profile.ride_frequency_hz
    heave = _clamp(
        -vertical_mps2 / (omega * omega) * intensity.suspension, profile.spring_travel_m
    )
    return BodyResponse(roll_deg=roll, pitch_deg=pitch, heave_m=heave)


def steering_angle_deg(
    curvature_per_m: float, speed_mps: float, profile: VehicleProfile
) -> float:
    """Centre front-wheel angle for a turn, in degrees, left positive.

    The geometric angle is atan(wheelbase x curvature); understeer adds a
    little more as cornering load rises, so a fast corner shows more lock
    than a slow one of the same radius.
    """

    validate_profile(profile)
    geometric = math.degrees(math.atan(profile.wheelbase * curvature_per_m))
    lateral_g = speed_mps * speed_mps * abs(curvature_per_m) / _GRAVITY_MPS2
    angle = geometric + math.copysign(profile.understeer_deg_per_g * lateral_g, curvature_per_m)
    return _clamp(angle, profile.max_steer_deg)


def max_centre_steer_deg(
    profile: VehicleProfile, inner_limit_deg: float, outer_limit_deg: float
) -> float:
    """Largest centre angle whose Ackermann inner and outer wheel angles fit.

    ``inner_limit_deg`` and ``outer_limit_deg`` are how far the wheel on the
    inside and the outside of the turn can steer before touching the body.
    """

    validate_profile(profile)
    ceiling = profile.max_steer_deg
    if inner_limit_deg <= 0.0 or outer_limit_deg <= 0.0:
        return 0.0

    def fits(angle: float) -> bool:
        inner, outer = ackermann_angles(
            angle, wheelbase=profile.wheelbase, track=profile.track, max_steer_deg=90.0
        )
        return inner <= inner_limit_deg and outer <= outer_limit_deg

    if fits(ceiling):
        return ceiling
    low, high = 0.0, ceiling
    for _ in range(40):
        middle = (low + high) / 2.0
        if fits(middle):
            low = middle
        else:
            high = middle
    return low


def plan_speed_profile(
    distances: list[float],
    curvatures: list[float],
    profile: VehicleProfile,
    *,
    target_speed_mps: float,
    grip_usage: float = 1.0,
) -> list[float]:
    """Grip-limited speeds from rest to rest along a sampled path.

    ``grip_usage`` is the share of the tyres' grip the driver uses: about
    0.35 for a relaxed drive, 0.55 for normal road driving and 1.0 at the
    limit. Braking and acceleration share that grip with cornering (a
    friction circle), so a driver finishes braking before the apex and
    feeds the throttle in on the way out. Acceleration also tapers with
    speed as power, not traction, becomes the limit.
    """

    validate_profile(profile)
    if len(distances) != len(curvatures) or len(distances) < 2:
        raise ValueError("distances and curvatures need the same two or more samples")
    if not 0.0 < grip_usage <= 1.0:
        raise ValueError("grip_usage must be within (0, 1]")
    lateral_limit = grip_usage * profile.lateral_grip_g * _GRAVITY_MPS2
    ceiling = min(target_speed_mps, profile.top_speed_mps)
    speeds = [
        ceiling if abs(curvature) <= 1e-9 else min(ceiling, math.sqrt(lateral_limit / abs(curvature)))
        for curvature in curvatures
    ]
    speeds[0] = 0.0
    speeds[-1] = 0.0

    def available(limit: float, speed: float, curvature: float) -> float:
        usage = min(1.0, speed * speed * abs(curvature) / lateral_limit)
        return limit * math.sqrt(max(0.0, 1.0 - usage * usage))

    for index in range(1, len(speeds)):
        speed = speeds[index - 1]
        drive = grip_usage * profile.acceleration_mps2 * max(
            0.0, 1.0 - (speed / profile.top_speed_mps) ** 2
        )
        accel = available(drive, speed, curvatures[index - 1])
        span = distances[index] - distances[index - 1]
        speeds[index] = min(speeds[index], math.sqrt(speed * speed + 2.0 * accel * span))
    for index in range(len(speeds) - 2, -1, -1):
        speed = speeds[index + 1]
        brake = available(grip_usage * profile.braking_mps2, speed, curvatures[index + 1])
        span = distances[index + 1] - distances[index]
        speeds[index] = min(speeds[index], math.sqrt(speed * speed + 2.0 * brake * span))
    return speeds


@dataclass(frozen=True)
class BodyPose:
    """Body offset from rest: heave up, nose-up pitch, left-side-up roll."""

    heave_m: float
    pitch_deg: float
    roll_deg: float


def _corner_rise(pose: BodyPose, corner: tuple[float, float]) -> float:
    forward, left = corner
    return (
        pose.heave_m
        + forward * math.tan(math.radians(pose.pitch_deg))
        + left * math.tan(math.radians(pose.roll_deg))
    )


def fit_body_pose(
    pose: BodyPose,
    corners: list[tuple[float, float]],
    compression_limits_m: list[float],
    extension_limit_m: float,
) -> BodyPose:
    """Keep every corner within its travel, preserving the lean if possible.

    ``corners`` are wheel positions (forward, left) from the body's pivot,
    in metres. A corner may drop by at most its compression limit, which is
    how close the wheel arch sits above that tyre, and rise by at most the
    extension limit. When the lean fits but the height does not, the body is
    raised or lowered instead, so the arch never meets the tyre; only a lean
    too large for the travel available is reduced.
    """

    if len(corners) != len(compression_limits_m):
        raise ValueError("one compression limit is needed per corner")
    if extension_limit_m < 0.0 or any(limit < 0.0 for limit in compression_limits_m):
        raise ValueError("travel limits must not be negative")

    def height_window(scale: float) -> tuple[float, float]:
        tilted = BodyPose(0.0, pose.pitch_deg * scale, pose.roll_deg * scale)
        tilts = [_corner_rise(tilted, corner) for corner in corners]
        lowest = max(-limit - tilt for limit, tilt in zip(compression_limits_m, tilts))
        highest = min(extension_limit_m - tilt for tilt in tilts)
        return lowest, highest

    scale = 1.0
    lowest, highest = height_window(scale)
    if lowest > highest:
        low, high = 0.0, 1.0
        for _ in range(40):
            middle = (low + high) / 2.0
            window = height_window(middle)
            if window[0] <= window[1]:
                low = middle
            else:
                high = middle
        scale = low
        lowest, highest = height_window(scale)
    heave = min(max(pose.heave_m, lowest), highest)
    return BodyPose(heave, pose.pitch_deg * scale, pose.roll_deg * scale)


@dataclass(frozen=True)
class SuspensionState:
    """Position and velocity of one spring-damper body degree of freedom."""

    value: float = 0.0
    velocity: float = 0.0


def suspension_step(
    state: SuspensionState,
    target: float,
    dt: float,
    *,
    natural_hz: float = 1.4,
    damping_ratio: float = 0.7,
) -> SuspensionState:
    """Advance a spring-damper toward ``target`` by ``dt`` seconds.

    ``body_response`` gives the pose the chassis would settle to; this
    supplies the motion between poses, so a change in acceleration eases the
    body over rather than snapping it there in a single frame. The step is
    split so each sub-step stays well inside the integrator's stable range,
    which keeps a long frame from exploding the response.
    """

    if not math.isfinite(dt) or dt < 0.0:
        raise ValueError("dt must be finite and non-negative")
    if not math.isfinite(natural_hz) or natural_hz <= 0.0:
        raise ValueError("natural_hz must be greater than zero")
    if not 0.0 <= damping_ratio <= 1.0:
        raise ValueError("damping_ratio must be between zero and one")

    omega = 2.0 * math.pi * natural_hz
    steps = max(1, math.ceil(dt * omega * 4.0))
    step = dt / steps
    value, velocity = state.value, state.velocity
    for _ in range(steps):
        acceleration = omega * omega * (target - value) - 2.0 * damping_ratio * omega * velocity
        velocity += acceleration * step
        value += velocity * step
    return SuspensionState(value=value, velocity=velocity)


@dataclass(frozen=True)
class AxlePose:
    """Where a two-axle vehicle sits when both axles are on the path."""

    position: tuple[float, float, float]
    heading_deg: float
    pitch_deg: float


def _cumulative(points: list[tuple[float, float, float]]) -> list[float]:
    distances = [0.0]
    for start, end in zip(points, points[1:]):
        distances.append(distances[-1] + math.dist(start, end))
    return distances


def _end_direction(points, from_end: bool) -> tuple[float, float, float]:
    pairs = zip(points[::-1][1:], points[::-1]) if from_end else zip(points, points[1:])
    for start, end in pairs:
        length = math.dist(start, end)
        if length > 1e-12:
            return tuple((e - s) / length for s, e in zip(start, end))
    raise ValueError("path needs two distinct points")


def point_at_distance(
    points: list[tuple[float, float, float]], distance_m: float
) -> tuple[float, float, float]:
    """Return the path point ``distance_m`` along a polyline.

    Distances before the start or past the end continue along the first or
    last segment, so a vehicle centred on either end of the path still has
    an axle to stand on.
    """

    path = [tuple(float(c) for c in point) for point in points]
    distances = _cumulative(path)
    if distance_m <= 0.0:
        direction = _end_direction(path, from_end=False)
        return tuple(p + d * distance_m for p, d in zip(path[0], direction))
    if distance_m >= distances[-1]:
        direction = _end_direction(path, from_end=True)
        overshoot = distance_m - distances[-1]
        return tuple(p + d * overshoot for p, d in zip(path[-1], direction))
    for index in range(len(path) - 1):
        if distances[index + 1] >= distance_m:
            span = distances[index + 1] - distances[index]
            blend = 0.0 if span <= 1e-12 else (distance_m - distances[index]) / span
            return tuple(
                s + blend * (e - s) for s, e in zip(path[index], path[index + 1])
            )
    return path[-1]


def axle_pose(
    points: list[tuple[float, float, float]], distance_m: float, wheelbase: float
) -> AxlePose:
    """Pose a vehicle whose centre is ``distance_m`` along the path.

    The rear axle stands half a wheelbase behind and the front axle half a
    wheelbase ahead, both on the path. Heading and slope come from the line
    between them rather than from the tangent at one point, so a corner knot
    turns the vehicle over its own length instead of in a single frame, and
    nothing accumulates that could drift away from the path.
    """

    if not math.isfinite(wheelbase) or wheelbase <= 0.0:
        raise ValueError("wheelbase must be greater than zero")
    rear = point_at_distance(points, distance_m - wheelbase / 2.0)
    front = point_at_distance(points, distance_m + wheelbase / 2.0)
    dx, dy, dz = (f - r for f, r in zip(front, rear))
    return AxlePose(
        position=tuple((f + r) / 2.0 for f, r in zip(front, rear)),
        heading_deg=math.degrees(math.atan2(dy, dx)),
        pitch_deg=math.degrees(math.atan2(dz, math.hypot(dx, dy))),
    )
