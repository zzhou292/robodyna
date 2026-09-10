# Optional actual force-stage acceleration capture

Set `NodalStateConfig::capture_force_stage_accelerations` before fresh extended
`StaggeredHalfKickStart` initialization with attached plain rigid groups. The
existing uniform initial group translation and zero spin scope remains in force.
Ordinary free, component-fixed and fully fixed nodes retain their current rules.
All other startup modes reject capture before any device allocation.

The optional scratch tail contains node A3/AR3 followed by group A3/AR3:
`6*(node_count+group_count)` doubles. It shares the existing scratch allocation,
does not extend either accepted/trial slab, and is included in the unchanged
1 MiB whole-owner device cap. The already allocated host state staging is proven
large enough at startup and reused for readback. No stepping allocation, group
selector or second clock is added. Disabled specializations omit all sink writes
and retain the original allocation sizes/count and numerical operations.

The enabled kernel stores actual ordinary accelerations at their computation,
and copies `PrimaryStepTrial` / `MemberStepTrial` accelerations from the qualified
native packets. It does not invert reactions, difference velocities, recompute
forces or repeat group recurrence. Fixed ordinary components store zero. Partial
scratch after a rejected advance is inaccessible through the public API.

`CopyPreparedForceStage(token, buffer, &prepared)` requires every node A/AR array
and enough source-associated group rows. It authenticates the complete token
through the actual owner, accepts only a completed pending candidate, validates
all destination/token ranges, stages the entire payload, and checks every value
before publishing any output. CUDA failure poisons the owner as existing reads
do; finite-value rejection discards the attempt and permits a fresh retry.
Commit/discard/new attempts make the old capture inaccessible. Capture has no
accepted readback. Calls are serialized with all other owner operations.

The sample is at `prepared.base_time`, using the actual kick's constrained A/AR.
Use pre-kick member/primary velocities and the updated axes from the **same**
`CopyPreparedRigidGroups` identity with the separate pure force-stage observable.
The first stage has DT1=0; later fixed-step stages use DT1=h. Capture alone is
neither collocated kinetic energy, the complete native ENCIN output, nor an
admission tolerance for physical energy balance.

## Owning CUDA qualification (2026-09-10)

Production7a716b9 and qualification57285f2 pass the complete existing rigid/nodal
CMake suite:93 functions in20 CTest groups, including13 new capture functions
and80 prior functions. Ten new functions execute CUDA: nine capture/readback/
capacity tests and one32-stage native-owner oracle. The other three new host/
native functions include64 force-stage packets. Guarded reports are
`crash-work/reports/force-stage-capture-build-1.json` and
`force-stage-capture-tests-1.json`; per-executable XML is in
`force-stage-capture-functions-1/`. Independent read-only review found no
remaining blocker. App observation composition and its live case gate remain
separate; this does not establish a physical energy acceptance tolerance.
