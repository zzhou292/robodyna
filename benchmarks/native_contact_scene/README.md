# Fixed triangle wall and moving shell patch

This package declares one small physical integration scene and exports its native
reference deck. It does not run a solver, provide native-calculated forces to TL,
or claim that the intended input flags have resolved to the supported profile.

`fixed_wall_patch.json` is the human-editable physical definition: eight wall
triangles, four moving QEPH shells, elastic steel and a separated initial state.
Units are explicit mm/tonne/s. `definition.py` validates its bounded scope;
`mesh.py` builds the shared physical roster; `cards.py` preserves exact-width
round-trip values; `native.py` emits the reference-only2024 input; `__main__.py`
writes a fresh directory and exact source/artifact hashes. No huge launcher or
general solver framework is introduced.

From the app worktree:

```
python3 -B -m benchmarks.native_contact_scene \
  benchmarks/native_contact_scene/fixed_wall_patch.json EXISTING_PARENT/NEW_CASE
python3 -B -m unittest benchmarks.native_contact_scene.test_scene
```

Input files are immutable/external-serialized for the export duration. Existing
destinations are rejected; unexpected I/O failures may leave diagnostic partial
files, which must be retained rather than overwritten. Declared input pins are
not evidence of resolved physics or a completed trajectory. Native reference
execution is separately guarded and source-observed before production admission.

The numerical input deliberately uses layered LAW1/NIP3/ITHICK1/ISMSTR2, matching
existing TL family work. Contact is nodes-to-surface against a finite fixed main
mesh. This first scene is elastic and does not claim deforming main-side normals,
vehicle self-contact or plastic impact. Those remain later integration gates.

Next: observe genuine startup normals/bisectors/adjacency, signed role/partner
identities, mass/inertia, coefficient/gap/removal producers and Engine branch/time
controls. Reimplement/reuse their producers and compare complete arrays; captured
native results remain expected test evidence. Integrate through the sole TL owner
and compare actual new impact, persistence, separation/recontact and rollback.
No CPU/GPU speed claim precedes complete matched numerical and timing runs.

Input syntax is checked against the pinned runtime reader configuration and
official [TYPE25](https://help.altair.com/hwsolvers/rad/topics/solvers/rad/inter_type25_starter_r.htm),
[shell](https://2025.help.altair.com/2025.1/hwsolvers/rad/topics/solvers/rad/prop_type1_shell_starter_r.htm)
and [surface](https://help.altair.com/hwsolvers/rad/topics/solvers/rad/surf_seg_starter_r.htm)
formats. Actual pinned source/Engine observations determine admitted behavior.
