# Vehicle Rig and Path Motion Design

## Goal

Turn a selected, existing 3ds Max vehicle into a non-destructive rig that
follows one validated open spline from start to finish with editable baked
animation, vehicle-aware motion, wheel motion, suspension response, and
manual animation controls.

## Scope

The production dialogue contains only two sections:

1. **Vehicle Rig**: bind a selected vehicle root and explicitly assign its
   body, four wheels, optional steering wheel, and optional axle/suspension
   parts. Choose Car, SUV, Pickup, Van, Bus, or Custom dimensions.
2. **Path and Motion**: bind one open spline, validate it, choose frame range,
   direction, target speed, and intensity settings, then bake editable keys.

The tool is MaxScript-native. It does not alter a vehicle's source geometry or
delete its existing controllers. It records the original parent and transform
of each assigned node before linking it under generated rig helpers.

## Repository cleanup

Remove the environment, lighting, and external-output code and its dedicated
tests from the repository:

- `server/frame.py`, `server/massing.py`, `server/maxbridge.py`,
  `server/osm.py`, `server/solar.py`, and `server/scene.py`
- `tests/test_frame.py`, `tests/test_massing.py`,
  `tests/test_massing_crossvalidation.py`, and `tests/test_osm.py`

Before removing `massing.py`, move its generic `Mesh` data type to
`server/geometry.py` and update `cars.py` to import it there. The retained
vehicle modules are `cars.py`, `raceanim.py`, `timeframe.py`, and the new rig
modules. The README will describe only the retained tool and tests.

## Motion model

`raceanim.py` remains the reference for path distance, heading, curvature,
corner speed, braking, traction, and body attitude. A new vehicle-dynamics
module will expose serializable vehicle presets and deterministic per-frame
samples for the MaxScript layer.

At each frame the sample contains path position, forward speed, longitudinal
acceleration, curvature, heading, body pitch, roll, heave, front-wheel steer,
rear-wheel steer, and four wheel spin distances. The speed cap is the minimum
of selected target speed, vehicle top speed, and a curve limit derived from
tyre grip and curvature. Deceleration is planned before the curve. The body
response uses bounded spring-damper offsets; intensity sliders multiply
response without exceeding preset-safe travel, roll, pitch, or steering lock.

Wheel steering uses the wheelbase and track to calculate different inner and
outer front-wheel angles. Each wheel's rotation derives from its travel
distance and selected tyre radius. The manual controls use separate additive
controller layers, so a user can adjust the baked animation without deleting
the generated path motion.

## Max rig interface

`maxscript/VehicleRigTool.ms` provides the dialogue and generated rig.

```text
VRIG_Master          square: translation and global offset
|- VRIG_Body          box: body heave, pitch, and roll
|- VRIG_Steering      circle: manual yaw and steering offset
|- VRIG_PathGuide     arrow: baked forward direction
`- VRIG_Wheel_*       circles: wheel spin and steer controllers
```

The dialogue must reject a missing selection, an assigned node outside the
selected vehicle hierarchy, absent wheels, an invalid wheel radius, or no
selected spline. It must report, without changing the scene, multiple spline
segments, closed paths, zero-length spans, duplicate knots, self-intersections,
abrupt kinks, and curve radii below the vehicle's steering capability.

The initial release accepts exactly one open spline. It supports normal or
reversed travel and creates baked, editable keys between the selected start
and end frames. It includes a `Remove Generated Rig` action that restores the
captured hierarchy and removes only nodes named with the `VRIG_` prefix.

## Verification

Python tests cover vehicle presets, steering geometry, wheel rotation,
curve-speed limits, braking, bounded intensity responses, and invalid input.
The existing car and race-animation tests remain green after the cleanup.

MaxScript verification includes a preflight-only mode that creates no scene
nodes, explicit generated-node naming, and a sample-scene test checklist for
a four-wheel vehicle and an open spline.

## Constraints

- Python 3.11+; keep offline tests runnable with the project virtual environment.
- 3ds Max 2027 is the supported host for the first release.
- Do not include generated-provenance content or attribution trailers.
- Do not make network requests, build terrain, change lighting, create cameras,
  render, or save files from the vehicle rig dialogue.
