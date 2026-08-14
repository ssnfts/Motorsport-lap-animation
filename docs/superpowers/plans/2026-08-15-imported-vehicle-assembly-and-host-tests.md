# Imported Vehicle Assembly and Host Tests Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Assemble a selected multi-part import non-destructively, create five named path fixtures, and validate actual Max bakes while retaining scene evidence.

**Architecture:** `VehicleRigTool.ms` gains selected-node assembly and spline-fixture helpers. Existing bake/removal functions remain responsible for controller creation and restoration. Python static tests guard native MaxScript API names and host-safety constraints.

**Tech Stack:** Python 3.13 with pytest; native 3ds Max 2027 MaxScript.

## Global Constraints

- Operate only on explicitly selected vehicle nodes; never combine, collapse, delete, rename, save, render, light, or network-enable source assets.
- Generated objects use `VRIG_`, record restoration state, and are removable through the existing rig removal action.
- Wheel clusters are recognised only from case-insensitive `wheelFL`, `wheelFR`, `wheelBL`, and `wheelBR` tokens.
- Five open spline fixtures persist in the scene and are labelled `VALID` or `EXPECTED REJECT`.

---

### Task 1: Implement selected import assembly

**Files:** Modify `tests/test_vehicle_rig_script.py`; modify `maxscript/VehicleRigTool.ms`.

**Interfaces:** `vrigClassifyWheelNode node -> integer` and `vrigAssembleSelectedVehicle vehicleName -> #(success, message, root, body, wheels)`.

- [ ] Write the failing test:

```python
def test_script_exposes_non_destructive_import_assembly(source: str) -> None:
    for token in ("Auto Assemble Imported Vehicle", "vrigAssembleSelectedVehicle", "vrigClassifyWheelNode", "wheelFL", "wheelFR", "wheelBL", "wheelBR"):
        assert token in source
```

- [ ] Run `.venv\Scripts\python.exe -m pytest tests/test_vehicle_rig_script.py::test_script_exposes_non_destructive_import_assembly -q`; expect failure because the assembly API is absent.
- [ ] Implement `vrigClassifyWheelNode` using `toLower node.name` and exact name-token searches. Before any mutation, reject an empty selection, absent wheel cluster, or ambiguous match. On success create a generated root plus four generated wheel parent helpers, record source state, parent all non-wheel meshes to root and wheel meshes to their generated wheel parent, then return bindings.
- [ ] Run `.venv\Scripts\python.exe -m pytest tests/test_vehicle_rig_script.py -q`; expect pass.
- [ ] Commit `maxscript/VehicleRigTool.ms` and `tests/test_vehicle_rig_script.py` with `feat: assemble imported vehicle meshes`.

### Task 2: Implement five persistent spline fixtures

**Files:** Modify `tests/test_vehicle_rig_script.py`; modify `maxscript/VehicleRigTool.ms`.

**Interface:** `vrigCreateTestPaths()` returns nodes named `VRIG_Test_Simple`, `VRIG_Test_Complex`, `VRIG_Test_UpHill`, `VRIG_Test_DownHill`, and `VRIG_Test_InvalidKnot`.

- [ ] Write the failing test:

```python
def test_script_declares_five_path_fixtures(source: str) -> None:
    for name in ("VRIG_Test_Simple", "VRIG_Test_Complex", "VRIG_Test_UpHill", "VRIG_Test_DownHill", "VRIG_Test_InvalidKnot", "EXPECTED REJECT"):
        assert name in source
```

- [ ] Run `.venv\Scripts\python.exe -m pytest tests/test_vehicle_rig_script.py::test_script_declares_five_path_fixtures -q`; expect failure because fixture creation is absent.
- [ ] Implement `vrigBuildOpenSpline name points` using `SplineShape`, `addNewSpline`, `addKnot`, and `updateShape`. Create four distinct valid paths and one path with adjacent equal points; add non-renderable labels. Never replace an existing fixture.
- [ ] Run `.venv\Scripts\python.exe -m pytest tests/test_vehicle_rig_script.py -q`; expect pass.
- [ ] Commit with `feat: add vehicle rig spline fixtures`.

### Task 3: Validate using the Dodge in 3ds Max

**Files:** Modify MaxScript/tests only if a host failure is reproduced and covered by a prior failing static test.

- [ ] Run `.venv\Scripts\python.exe -m pytest tests -q`; expect pass.
- [ ] Select only Dodge meshes, run **Auto Assemble Imported Vehicle**, and verify all source meshes descend from the generated root or wheel parents.
- [ ] Run **Create Five Spline Tests**. Preflight Simple, Complex, UpHill, and DownHill with no errors. Confirm InvalidKnot reports `duplicate_knot` or `zero_length_segment` and creates no bake helpers/keys.
- [ ] Bake each valid spline from frame 0 to 160, restoring between cases. Verify master motion and wheel-controller keys.
- [ ] Reassemble and leave the final verified rig on `VRIG_Test_Complex`, with all labelled spline fixtures retained.
