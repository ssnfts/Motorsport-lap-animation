# Imported Vehicle Assembly and Spline Test Design

## Goal

Make the native Vehicle Rig dialog able to rig a purchased multi-part vehicle without destructive mesh combining, then exercise it in 3ds Max with five named spline cases.

## Vehicle assembly

The dialog will add an **Auto Assemble Imported Vehicle** action. It accepts the currently selected imported vehicle meshes and creates generated parent nodes only:

- `VRIG_<vehicle>_Root` is the non-destructive vehicle root.
- `VRIG_<vehicle>_Wheel_FL`, `_FR`, `_RL`, and `_RR` are wheel-cluster parents.
- Wheel membership is inferred only from unambiguous case-insensitive name tokens: `wheelFL`, `wheelFR`, `wheelBL`, and `wheelBR`. Ambiguous or missing clusters stop assembly and report the affected names.
- All selected non-wheel meshes become children of the generated root. Wheel meshes become children of their corresponding generated wheel parent, which itself becomes a child of the root.

No source mesh is collapsed, combined, deleted, renamed, or frozen. The original parent and transform of every changed node are recorded using the existing `VRIG_` restoration properties, so **Remove Generated Rig** restores the import.

After assembly, the dialog fills Vehicle Root, Body, and the four wheel bindings from the generated nodes. The body binding is the root, ensuring every non-wheel vehicle part follows the master controller.

## Spline test scene

The live Max test creates a `VRIG_TestPaths` collection with five labelled, open spline nodes:

1. `VRIG_Test_Simple` — planar, smooth, valid.
2. `VRIG_Test_Complex` — planar S-turns, valid.
3. `VRIG_Test_UpHill` — valid path with increasing Z values.
4. `VRIG_Test_DownHill` — valid path with decreasing Z values.
5. `VRIG_Test_InvalidKnot` — open path containing a repeated/near-zero-length knot, expected to be rejected by preflight.

Each path receives a non-renderable scene label identifying its expected result. Valid cases are preflighted and baked on the assembled Dodge. The rig is restored between valid cases, so each test begins from the preserved import state. The invalid case must create no bake helpers or animation keys.

## Verification and evidence

Automated tests will cover the MaxScript contract for the assembly action, wheel-token rules, original-state restoration, test-path names, and invalid-path no-bake behavior. In Max, the test procedure will record labels and a listener report with the path name, preflight result, bake result, and any warning. The final rig remains on the complex valid path; the remaining test splines and markers remain in the scene as evidence.

## Constraints

- Native MaxScript only; no network, AI, rendering, lighting, environment, or output generation features.
- The tool operates on selected nodes only and fails rather than guessing missing or ambiguous wheel clusters.
- Generated nodes use the `VRIG_` prefix and can be removed through the existing restoration action.
