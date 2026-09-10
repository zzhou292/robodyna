# Actual six-part source GPU free-flight gate

This focused qualification composes existing `SourceAssemblyBindings`, TL's
active QEPH/T3 participants and plastic catalog, one `FENodalState`, and one
`ShellBatchPublication`. It retains the source-neutral contact geometry. It
introduces no application solver, physical mass formula, independent clock,
per-step production allocation, contact force, renderer, or crash trajectory.

The exact inventory is authenticated by the named pinned reader: 804 QEPH and
111 T3 parents, 1030 native nodes, six material/section declarations and two
curves with 63 samples. All coordinates, original ELFORM, source EID/PID/family
indices, every catalog curve and rate declaration are checked. Native inverse
mass and isotropic inertia are copied back from the actual owner and checked
against the complete TL startup binding. The six internal rigid groups and
released external frontier remain retained source data. **No internal group is
attached or dynamically enforced in this gate.**

Startup is 8 m/s uniform +X translation and zero spin. Four intervals at
`1/67108864` seconds each test source geometry storage and free-flight recurrence.
Every parent uses its own catalog parameters in a direct host layered-J2 value
operation at each measured CUDA endpoint. Tests compare complete shell histories,
forces/couples, kinematics, diagnostics and section point states, including the
last source parents. Existing TL qualification field utilities are reused;
this host path checks resident storage and mapping, and does not constitute a
new independent validation of material or shell equations.

The analytic rigid-flight bounds reuse the existing original-part coefficient
`2e-13*(steps+1)` with coordinate, velocity and shortest-edge dimensional scales.
Plastic strain/work remain zero. A rejected complete candidate preserves every
accepted nodal/shell/section field; retry produces identical fields. The sole
owner and both material slabs publish once. Allocation sizes/counts stay fixed.

Configure this directory explicitly with `ROBO_DYNA_TL_ROOT`, `Chrono_DIR`,
`ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY` and `CMAKE_CUDA_ARCHITECTURES=120`. The owning
target is `robo_dyna_source_assembly_flight_check`; CTest is `source_assembly_flight`.
GPU discovery must succeed; missing CUDA does not produce a passing skip.
Use the serialized workstation resource guard. No build/test execution was done
by the implementation agent; the main integration owner runs qualification.

Passing this gate supports original-source GPU capacity and free flight only.
Internal group recurrence/force transfer and mesh-wall contact integration remain
separate gates before connected impact, longer trajectories and video output.
