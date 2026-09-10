# Authenticated assembly startup composition

`SourceAssemblyBindings::Prepare` composes the immutable modelio source adapters with TL's native shell binding, complete per-parent material catalog and internal nodal-rigid startup model. It requires an explicit nonzero source-instance ID and named direct-import rate policy. The full source identity remains owned alongside that instance; the instance ID is not a source hash or simulation authority.

The case owns no FE state, mass formula, clock or dynamics. Its three startup stages publish together through a shared immutable handle; assignment is deleted and moving retains a valid source handle. Source geometry is never dropped, triangulated or changed to pass admission. A native shell failure reports the exact family, EID, PID, family index and native/binding status.

The per-part ledger adds the already prepared native parent/local mass and all three inertia channels in source traversal order. The authoritative global nodal union retains TL's QEPH-then-T3 order; tiny roundoff differences between diagnostic reductions are expected. The six-part gate can independently compare part ledgers through its disjoint global-node inventory.

Rigid member staging crosschecks the original source ID, global index and coordinate bits against the prepared shell binding before any rigid startup. Complete member ranges are reserved before creating borrowed group pointers. TL receives the native nodal mass, native total J and both partition channels, with explicit original t/mm source units. Its generated-primary and principal-inertia corrections remain separate ledger channels; this layer never appends primary nodes or counts group mass again in the physical source inventory.

Only complete internal groups enter the constraint startup model. Four outgoing nodal-rigid groups, 13 spotwelds, 37 frontier nodes and the released one-sided tied source scope remain in the owned source boundary declaration. This is the extracted component's explicit released boundary, not full-vehicle closure or an already executed constraint model. A source with no internal groups has no rigid startup model.

Six host tests exercise all actual 915 native parents / 1030 nodes, the six MID/section mappings and 63 curve points, all 76 internal rigid members, per-part M/J and group tensor/regularization ledgers, owned lifetimes, independent stage caps, and an isolated invalid-reference fixture with exact source diagnostics. Test properties export the native ledgers into gtest XML. The immutable frozen source artifact is never rewritten.

Standalone qualification uses this directory as the CMake source, with explicit `Chrono_DIR`, `ROBO_DYNA_TL_ROOT` and `ROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY`. The top-level opt-in is `ROBO_DYNA_ENABLE_SOURCE_ASSEMBLY_STARTUP_CHECKS`. Source reader/adapters remain separate modelio targets; TL startup libraries are linked through their existing CMake fragments.

At handoff only the bounded syntax gate has run: one affinity CPU, 512 MiB virtual-address cap, 1.69 seconds and 225348 KiB peak RSS. Root owns the numerical host gate and integrated checks. No larger CUDA capacity, owner or crash trajectory is admitted here.
