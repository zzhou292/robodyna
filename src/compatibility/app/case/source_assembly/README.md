# Authenticated assembly startup composition

`SourceAssemblyBindings::Prepare` composes the immutable modelio source adapters with TL's native shell binding, complete per-parent material catalog and internal nodal-rigid startup model. It requires an explicit nonzero source-instance ID and named direct-import rate policy. The full source identity remains owned alongside that instance; the instance ID is not a source hash or simulation authority.

The case owns no FE state, mass formula, clock or dynamics. Its three startup stages publish together through a shared immutable handle; assignment is deleted and moving retains a valid source handle. Source geometry is never dropped, triangulated or changed to pass admission. A native shell failure reports the exact family, EID, PID, family index and native/binding status.

The per-part ledger adds the already prepared native parent/local mass and all three inertia channels in source traversal order. The authoritative global nodal union retains TL's QEPH-then-T3 order; tiny roundoff differences between diagnostic reductions are expected. The six-part gate can independently compare part ledgers through its disjoint global-node inventory.

Rigid member staging crosschecks the original source ID, global index and coordinate bits against the prepared shell binding before any rigid startup. Complete member ranges are reserved before creating borrowed group pointers. TL receives the native nodal mass, native total J and both partition channels, with explicit original t/mm source units. Its generated-primary and principal-inertia corrections remain separate ledger channels; this layer never appends primary nodes or counts group mass again in the physical source inventory.

Only complete internal groups enter the constraint startup model. Four outgoing nodal-rigid groups, 13 spotwelds, 37 frontier nodes and the released one-sided tied source scope remain in the owned source boundary declaration. This is the extracted component's explicit released boundary, not full-vehicle closure or an already executed constraint model. A source with no internal groups has no rigid startup model.

Six host tests exercise all actual 915 native parents / 1030 nodes, the six MID/section mappings and 63 curve points, all 76 internal rigid members, per-part M/J and group tensor/regularization ledgers, owned lifetimes, independent stage caps, and an isolated invalid-reference fixture with exact source diagnostics. Test properties export the native ledgers into gtest XML. The immutable frozen source artifact is never rewritten.

Standalone qualification uses this directory as the CMake source, with explicit `Chrono_DIR`, `ROBO_DYNA_TL_ROOT` and `ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY`. The top-level opt-in is `ROBO_DYNA_ENABLE_SOURCE_ASSEMBLY_STARTUP_CHECKS`. Source reader/adapters remain separate modelio targets; TL startup libraries are linked through their existing CMake fragments.

At handoff only the bounded syntax gate has run: one affinity CPU, 512 MiB virtual-address cap, 1.69 seconds and 225348 KiB peak RSS. Root owns the numerical host gate and integrated checks. No larger CUDA capacity, owner or crash trajectory is admitted here.

All six startup tests pass against the frozen six-part inventory (2026-09-10).
Every actual native reference and material declaration prepares successfully.
Native structural mass is 1.8021908802155335 kg; all six complete internal
groups preserve their 76 original members. No group activates the source
principal-inertia correction, while native shell added inertia and generated
primary regularizers remain explicit ledger channels. Evidence:
`crash-work/reports/source-assembly-startup-{configure,build,tests}-1`; the
GTest XML records the full per-part and per-group M/J channels.
This is immutable host startup evidence; it does not qualify a connected crash trajectory.

## Complete assembly wall preparation

`SourceAssemblyWallSetup` retains authenticated bindings, all 915 parents and
1030 native nodes, the six complete internal groups, and an explicitly declared
`ReleasedExternalConnections` boundary. It owns the original finite wall and its
declared X placement through immutable shared handles. Copy and move construction
preserve both handles; assignment is disabled. The setup owns no nodal state,
inverse mass array, clock, force evaluation or simulation driver.

The bounded startup supports uniform positive X velocity and zero spin. Its
initial kinetic certificate names two distinct metrics: all original native
nodes, and ordinary native nodes plus each complete group's total mass once.
The latter includes the native generated primary through the group's existing
mass property; the primary is never added a second time. Per-group native and
aggregate energy enclosures and the generated-primary mass ledger remain
available for inspection. At 8 m/s the nominal native and aggregate energies
are respectively 57.670108166897002 J and 57.670108166896981 J; their small
reduction-order difference does not change their physical scope.

The source-neutral `case/wall_penalty` target owns uniform translation energy,
directed penalty certification, represented gap and projected motion bounds.
The old source-part setup delegates to these same utilities while retaining
its public API, original native-node reduction order and fixed penalty design.
The complete assembly's minimum certified nodal area is
1.6918079278568485e-5 m². Its default area floor of 1e-5 m², 1.5 mm design
penetration and 1.10 energy factor produce a fixed stiffness per area of
5.6388550207634268e12 at 8 m/s. This initial energy inequality and the separate
native-mass contact rate diagnostic do not certify a coupled constrained
timestep or reconstruct time-staggered kinetic energy.

Wall coverage uses the placed mesh's exact X plane for both physical motion-box
X endpoints, with outward-rounded Y/Z margins. The represented leading gap must
remain strictly positive. Initial gap is explicit: the host test uses 5 µm to
permit a later short contact smoke, while the default remains 20 mm. The wall's
62 source vertices and 100 contact triangles remain associated with their
original source IDs; source shell coordinates are never translated or omitted.

`MakeDeviceConfig` exports the certified law and geometry scope after checking
a complete fresh stamp and exact rigid-group descriptor. This is a declaration
check, not live-owner authentication. TL retains allocation-fit admission,
native coefficient validation and live owner/token authentication. The owning
CMake include is `SourceAssemblyWallSetup.cmake`, with target
`robo_dyna_source_assembly_wall_setup`.

Author qualification passed seven complete assembly/shared-value host tests
and three existing source-part setup regressions. Coverage includes source and
boundary preservation, both energy scopes, finite mesh coverage, owned
lifetimes, malformed/late limits, failure-preserving publication and config
scope/alias rejection. The checks reused qualified host libraries under one
affinity CPU and a 512 MiB virtual-address cap; peak child RSS was 265096 KiB
for the assembly gate and 251764 KiB for the part regression. XML evidence is
`/tmp/source-assembly-wall-setup-check.xml` and
`/tmp/source-part-wall-shared-certification-check.xml`. Owning CMake and CUDA
integration qualification remain separate; this gate runs no GPU trajectory.
