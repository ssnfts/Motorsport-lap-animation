# Vehicle Rig and Path Motion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a native 3ds Max vehicle-rig dialogue that binds an existing vehicle, validates one open spline, and bakes realistic editable vehicle motion.

**Architecture:** Keep deterministic geometry, vehicle profiles, motion sampling, and spline preflight in small Python modules with offline tests. A standalone MaxScript tool owns all 3ds Max UI and scene mutation; it never depends on the removed bridge or environment modules.

**Tech Stack:** Python 3.11, pytest, 3ds Max 2027 MaxScript.

## Global Constraints

- Support exactly one selected existing vehicle root and one open spline in the first release.
- Preserve source vehicle geometry and existing controllers; generated scene nodes use the `VRIG_` prefix only.
- Keep all Python tests offline and run them with `.venv\Scripts\python.exe -m pytest tests -q`.
- Do not make network requests, build terrain, change lighting, create cameras, render, or save files from the rig dialogue.
- Do not include generated-provenance content or attribution trailers.

---

## File structure

| Path | Responsibility |
|---|---|
| `server/geometry.py` | Generic triangle mesh data type retained by car proxies. |
| `server/cars.py` | Generic proxy geometry; imports `Mesh` only from `geometry.py`. |
| `server/vehicle_dynamics.py` | Vehicle presets, validation, speed caps, steering, wheel spin, and bounded body response. |
| `server/spline_preflight.py` | Host-independent open-path validation and warning report. |
| `maxscript/VehicleRigTool.ms` | Max dialogue, explicit node binding, helpers, controller layering, key baking, and removal. |
| `tests/test_geometry.py` | Retained mesh and car-proxy compatibility. |
| `tests/test_vehicle_dynamics.py` | Preset and per-frame dynamics contract. |
| `tests/test_spline_preflight.py` | Path warning and rejection contract. |

### Task 1: Isolate vehicle geometry and remove excluded modules

**Files:**
- Create: `server/geometry.py`
- Create: `tests/test_geometry.py`
- Modify: `server/cars.py:33`
- Modify: `README.md`
- Delete: `server/frame.py`, `server/massing.py`, `server/maxbridge.py`, `server/osm.py`, `server/solar.py`, `server/scene.py`
- Delete: `tests/test_frame.py`, `tests/test_massing.py`, `tests/test_massing_crossvalidation.py`, `tests/test_osm.py`

**Interfaces:**
- Produces: `geometry.Mesh(verts, faces, name, metadata, uvs)` with `vertex_count`, `face_count`, `bounds()`, and `volume()`.
- Consumes: no removed module.

- [ ] **Step 1: Write the failing compatibility tests**

```python
from geometry import Mesh
import cars

def test_mesh_keeps_car_proxy_contract():
    mesh = Mesh(verts=[(0.0, 0.0, 0.0)], name="probe")
    assert mesh.name == "probe"
    assert mesh.vertex_count == 1
    assert cars.car_mesh("car").vertex_count > 0

def test_empty_mesh_has_no_bounds():
    with pytest.raises(ValueError, match="empty mesh"):
        Mesh().bounds()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_geometry.py -q`

Expected: import failure because `geometry` does not exist.

- [ ] **Step 3: Move the complete Mesh dataclass and update the retained import**

```python
@dataclass
class Mesh:
    verts: list[tuple[float, float, float]] = field(default_factory=list)
    faces: list[tuple[int, int, int]] = field(default_factory=list)
    name: str = "mesh"
    metadata: dict = field(default_factory=dict)
    uvs: list[tuple[float, float]] = field(default_factory=list)

    @property
    def vertex_count(self) -> int:
        return len(self.verts)

    @property
    def face_count(self) -> int:
        return len(self.faces)
```

Copy the existing `bounds()` and signed-volume implementation verbatim, changing its exception to `ValueError`. Change `cars.py` to `from geometry import Mesh`. Delete only the listed environment/output files and their dedicated tests. Rewrite the README as a vehicle animation and rigging project without data, lighting, or bridge claims.

- [ ] **Step 4: Run retained geometry and motion tests**

Run: `.venv\Scripts\python.exe -m pytest tests/test_geometry.py tests/test_cars.py tests/test_raceanim.py -q`

Expected: PASS.

- [ ] **Step 5: Commit**

```powershell
git add README.md server/geometry.py server/cars.py tests/test_geometry.py
git add -u
git commit -m "refactor: focus project on vehicle motion"
```

### Task 2: Add vehicle profiles and deterministic dynamics

**Files:**
- Create: `server/vehicle_dynamics.py`
- Create: `tests/test_vehicle_dynamics.py`

**Interfaces:**
- Produces: `VehicleProfile`, `VEHICLE_PRESETS`, `validate_profile`, `curve_speed_limit`, `ackermann_angles`, `wheel_rotation_degrees`, and `body_response`.
- Consumes: Python `math` only; no Max or network dependency.

- [ ] **Step 1: Write the failing profile and wheel-motion tests**

```python
from vehicle_dynamics import VEHICLE_PRESETS, ackermann_angles, wheel_rotation_degrees

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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_vehicle_dynamics.py -q`

Expected: import failure because `vehicle_dynamics` does not exist.

- [ ] **Step 3: Implement the narrow public API**

```python
@dataclass(frozen=True)
class VehicleProfile:
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

def curve_speed_limit(radius_m: float, profile: VehicleProfile) -> float:
    return min(profile.top_speed_mps, math.sqrt(max(radius_m, 0.0) * 9.80665 * profile.lateral_grip_g))

def wheel_rotation_degrees(distance_m: float, tyre_radius_m: float) -> float:
    return math.degrees(distance_m / tyre_radius_m)
```

Implement `ackermann_angles` with `atan(wheelbase / (turn_radius +/- track / 2))`, preserve the turn sign, and clamp both values to `max_steer_deg`. Implement `body_response` as a pure bounded output from acceleration, lateral acceleration, and intensity settings; it must not exceed profile travel, roll, or pitch limits.

- [ ] **Step 4: Add tests for invalid profiles, speed cap, clamp, and bounded response**

```python
def test_body_response_never_exceeds_profile_limits():
    profile = VEHICLE_PRESETS["suv"]
    response = body_response(profile, longitudinal_mps2=50.0, lateral_mps2=50.0, intensity=Intensity.all_max())
    assert abs(response.roll_deg) <= profile.max_roll_deg
    assert abs(response.pitch_deg) <= profile.max_pitch_deg
    assert abs(response.heave_m) <= profile.spring_travel_m
```

- [ ] **Step 5: Run focused tests and commit**

Run: `.venv\Scripts\python.exe -m pytest tests/test_vehicle_dynamics.py -q`

Expected: PASS.

```powershell
git add server/vehicle_dynamics.py tests/test_vehicle_dynamics.py
git commit -m "feat: add vehicle dynamics profiles"
```

### Task 3: Add spline preflight

**Files:**
- Create: `server/spline_preflight.py`
- Create: `tests/test_spline_preflight.py`

**Interfaces:**
- Produces: `SplineIssue(code, severity, message, index)` and `preflight_open_path(points, profile) -> list[SplineIssue]`.
- Consumes: a sequence of `(x, y, z)` points and `VehicleProfile`.

- [ ] **Step 1: Write the failing warning tests**

```python
from spline_preflight import preflight_open_path
from vehicle_dynamics import VEHICLE_PRESETS

def test_duplicate_knot_is_an_error():
    issues = preflight_open_path([(0, 0, 0), (1, 0, 0), (1, 0, 0)], VEHICLE_PRESETS["car"])
    assert any(issue.code == "duplicate_knot" and issue.severity == "error" for issue in issues)

def test_tight_curve_is_reported_before_baking():
    issues = preflight_open_path([(0, 0, 0), (1, 0, 0), (1, 0.1, 0)], VEHICLE_PRESETS["bus"])
    assert any(issue.code == "steering_limit" for issue in issues)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_spline_preflight.py -q`

Expected: import failure because `spline_preflight` does not exist.

- [ ] **Step 3: Implement deterministic issue detection**

```python
@dataclass(frozen=True)
class SplineIssue:
    code: str
    severity: Literal["warning", "error"]
    message: str
    index: int | None = None

def preflight_open_path(points, profile):
    if len(points) < 2:
        return [SplineIssue("too_short", "error", "Path needs at least two distinct points")]
    return []
```

Treat fewer than two distinct points, zero-length segments, and duplicate neighbours as errors. Treat sharp kinks, self-intersections, and steering-limit failures as warnings that require an explicit acknowledgement in MaxScript before baking. Use 2D segment intersection on non-adjacent spans and curvature from three consecutive points.

- [ ] **Step 4: Verify focused tests and commit**

Run: `.venv\Scripts\python.exe -m pytest tests/test_spline_preflight.py tests/test_vehicle_dynamics.py -q`

Expected: PASS.

```powershell
git add server/spline_preflight.py tests/test_spline_preflight.py
git commit -m "feat: validate vehicle motion paths"
```

### Task 4: Create the native 3ds Max rig and dialogue

**Files:**
- Create: `maxscript/VehicleRigTool.ms`
- Create: `tests/test_vehicle_rig_script.py`

**Interfaces:**
- Consumes: selected root node, explicitly selected body/wheel nodes, one selected open `SplineShape`, vehicle profile values, frame range, direction, and intensity values.
- Produces: `VRIG_Master`, `VRIG_Body`, `VRIG_Steering`, `VRIG_PathGuide`, four `VRIG_Wheel_*` controls, generated keys, and a removable rig record.

- [ ] **Step 1: Write a failing static contract test**

```python
def test_maxscript_exposes_required_safety_actions():
    source = (ROOT / "maxscript" / "VehicleRigTool.ms").read_text(encoding="utf-8")
    for token in ("Create Vehicle Rig", "Preflight", "Bake Animation", "Remove Generated Rig", "VRIG_Master"):
        assert token in source
```

- [ ] **Step 2: Run test to verify it fails**

Run: `.venv\Scripts\python.exe -m pytest tests/test_vehicle_rig_script.py -q`

Expected: failure because the MaxScript source does not exist.

- [ ] **Step 3: Implement the MaxScript command and rollout**

```maxscript
rollout VehicleRigRollout "Create Vehicle Rig" width:360
(
    pickbutton rootPick "Pick Vehicle Root" width:160
    pickbutton bodyPick "Pick Body" width:160
    pickbutton splinePick "Pick Open Spline" width:160
    button preflightButton "Preflight" width:160
    button bakeButton "Bake Animation" width:160 enabled:false
    button removeButton "Remove Generated Rig" width:160
)
```

Require explicit bindings for body and four wheels before enabling Bake. Create helpers with the exact `VRIG_` names. Store `originalParent` and `originalTransform` in custom attributes on bound nodes. Use position and rotation list controllers: generated keys are the base layer and manual helper offsets are the top layer. Bake one key per frame from start through end, use spline arc-length position and tangent for translation/yaw, profile steering for front wheels, travel/radius for wheel spin, and bounded body response for the body helper. `Remove Generated Rig` restores recorded parents/transforms and deletes only generated `VRIG_` nodes.

- [ ] **Step 4: Extend the static test for generated-node-only deletion**

```python
def test_script_limits_deletion_to_generated_nodes():
    source = (ROOT / "maxscript" / "VehicleRigTool.ms").read_text(encoding="utf-8")
    assert 'matchPattern node.name pattern:"VRIG_*"' in source
    assert "originalParent" in source
    assert "originalTransform" in source
```

- [ ] **Step 5: Run contract tests and commit**

Run: `.venv\Scripts\python.exe -m pytest tests/test_vehicle_rig_script.py -q`

Expected: PASS.

```powershell
git add maxscript/VehicleRigTool.ms tests/test_vehicle_rig_script.py
git commit -m "feat: add native vehicle rig dialogue"
```

### Task 5: Integrate, document, and verify

**Files:**
- Modify: `README.md`
- Modify: `docs/superpowers/specs/2026-08-11-vehicle-rig-design.md`

**Interfaces:**
- Produces: documented setup and host verification checklist.

- [ ] **Step 1: Document exact setup and host verification**

```markdown
1. In 3ds Max 2027, run `maxscript/VehicleRigTool.ms`.
2. Pick the vehicle root, body, and four wheel nodes.
3. Pick one open spline, select a preset, and run Preflight.
4. Resolve errors; acknowledge warnings only after reviewing them.
5. Bake Animation, inspect helper layers, and use Remove Generated Rig to restore the vehicle.
```

- [ ] **Step 2: Run every retained offline test**

Run: `.venv\Scripts\python.exe -m pytest tests -q`

Expected: PASS with no skipped tests introduced by this feature.

- [ ] **Step 3: Run source checks**

Run: `git diff --check && git status --short`

Expected: no whitespace errors; only intended modified files before staging.

- [ ] **Step 4: Commit and request host validation**

```powershell
git add README.md docs/superpowers/specs/2026-08-11-vehicle-rig-design.md
git commit -m "docs: explain vehicle rig workflow"
```

Record that offline tests prove calculation and script contract only. Validate actual helper creation, picker behaviour, controller layering, and scene restoration manually in 3ds Max 2027 before claiming host compatibility.

