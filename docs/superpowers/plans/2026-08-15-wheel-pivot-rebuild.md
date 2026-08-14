# Wheel Pivot Rebuild Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make imported grouped wheels spin and steer without moving away from the vehicle.

**Architecture:** Imported group heads stay passive. Each wheel receives a generated `VRIG_WheelPivot_*` helper at its measured visual centre and a child `VRIG_WheelSpin_*` helper. Steering keys the pivot and spin keys the child; no native group head receives a replacement controller.

**Tech Stack:** Native 3ds Max 2027 MaxScript and Python pytest.

## Global Constraints

- Do not combine, rename, collapse, reset, or directly animate imported mesh or group-head transforms.
- Keep the `VRIG_` cleanup contract.
- Verify frames 0, 90, 155, and 300 in the Max viewport.

---

### Task 1: Add controller-safe wheel helpers

**Files:** Modify `tests/test_vehicle_rig_script.py` and `maxscript/VehicleRigTool.ms`.

- [ ] Write a failing static test requiring `VRIG_WheelPivot_`, `VRIG_WheelSpin_`, `wheelPivots`, and `wheelSpinControllers`, and prohibiting `vrigLayeredControllers wheelGroup`.
- [ ] Run the focused pytest test and verify it fails.
- [ ] Create a pivot at each measured wheel centre, create an identically placed spin child, and parent the imported wheel assembly below the spin helper.
- [ ] Give layered controllers only to the generated pivot and spin helpers; run the focused and full pytest suites.
- [ ] Commit with `fix: use generated wheel pivots for motion`.

### Task 2: Separate steering from wheel spin

**Files:** Modify `tests/test_vehicle_rig_script.py` and `maxscript/VehicleRigTool.ms`.

- [ ] Write a failing static test requiring `wheelPivotControllers[index][2].value` and `wheelSpinControllers[index][2].value`.
- [ ] Run the focused pytest test and verify it fails.
- [ ] Bake `(eulerAngles 0 0 steer)` to pivot helpers and `(eulerAngles 0 (-wheelSpin[index]) 0)` to spin helpers.
- [ ] Run the full pytest suite and `git diff --check`.
- [ ] Commit with `fix: separate wheel steering from spin`.

### Task 3: Live verification and recovery

**Files:** Modify source only if a new live failure is reproduced and covered first by a failing test.

- [ ] Assemble the selected Dodge and bake `VRIG_Test_Complex` over frames 0–300.
- [ ] Inspect frames 0, 90, 155, and 300; compare each source-wheel evaluated centre with its generated pivot.
- [ ] Remove the rig; confirm no `VRIG_Dodge*` nodes or `VRIG_bound` metadata remain.
- [ ] Rebuild, leave the corrected rig at frame 0, run full pytest, commit, and push `codex/vehicle-rig`.
