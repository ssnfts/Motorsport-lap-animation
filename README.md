# Motorsport vehicle animation and rigging

Build realistic vehicle motion from a circuit centre-line. The project provides
grip-limited speed profiles, braking and traction sweeps, editable lap timing,
and generic open-wheel vehicle proxy geometry. It is the motion foundation for
a 3ds Max vehicle-rigging workflow.

## Layout

```
server/      vehicle geometry and motion modules
maxscript/   VehicleRigTool.ms, the 3ds Max rig dialogue
tests/       automated checks
```

## Rig a vehicle in 3ds Max

1. Run `maxscript/VehicleRigTool.ms` in 3ds Max 2027.
2. Select an imported vehicle's meshes and press **Auto Assemble Imported
   Vehicle**, or pick the root, body and four wheels yourself. Wheel parts are
   found by the `wheelFL`, `wheelFR`, `wheelBL` and `wheelBR` name tokens;
   parts named `caliper` or `brake` steer with their wheel but do not spin.
3. The dialogue measures the vehicle from its meshes: tyre radius, wheelbase,
   ground plane and facing. If the scene's unit setting would make the
   wheelbase implausible, it fits *Scene units per metre* to the preset
   instead and says so.
4. Pick one open spline (or press **Create Five Spline Tests**) and run
   **Preflight**. Errors name the knot at fault; warnings must be accepted.
5. **Bake Animation**. With *Match frame range to drive time* ticked, the
   end frame follows the physical drive time; untick it to fit a fixed range.
6. **Remove Generated Rig** restores every node's original parent and
   transform and deletes only `VRIG_` nodes (test splines are kept).

Baking again replaces the previous bake. Keys go on each helper's
*Generated Path* layer; the *Manual Offset* layer is free for hand edits.

## Run the tests

```
python -m pytest tests -q
```

## Requirements

Python 3.11+. The tests are offline and need no network connection.

## Scope

The Python modules hold the host-independent maths the MaxScript mirrors:
presets, steering, spring-damper body motion, and the two-axle path pose.
The offline tests check that maths and the script's contract; behaviour in
the viewport was verified on 3ds Max 2027 only.
