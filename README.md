# Motorsport vehicle animation and rigging

Build realistic vehicle motion from a circuit centre-line. The project provides
grip-limited speed profiles, braking and traction sweeps, editable lap timing,
and generic open-wheel vehicle proxy geometry. It is the motion foundation for
a 3ds Max vehicle-rigging workflow.

## Layout

```
server/   vehicle geometry and motion modules
tests/    automated checks
```

## Run the tests

```
python -m pytest tests -q
```

## Requirements

Python 3.11+. The tests are offline and need no network connection.

## Scope

The current modules focus on vehicle geometry and motion planning. A planned
rigging interface will bind a selected vehicle's parts and bake the resulting
motion in 3ds Max.
