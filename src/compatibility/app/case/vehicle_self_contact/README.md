# Vehicle self-contact composition

This module binds the authenticated selected vehicle surface to TL-FEA's
`SelfContactTransaction`. It installs a contribution into the existing
`VehiclePhysicalDynamics` owner; it owns no second solver clock and implements
no duplicate contact algorithm. The application-wide source map is in
[ARCHITECTURE.md](../../docs/ARCHITECTURE.md).

## Entry points and ownership

| Source | Responsibility |
| --- | --- |
| `SelectedSelfContactSource.*` | Resolve and authenticate the original selected shell surface |
| `VehicleSelfContactSetup.*` | Retain physical backing, fixed facets, source provenance and active-use bindings |
| `VehicleSelfContactInitialCensus.*`, `InitialCensusValues.*` | Initial source/filter census and bounded diagnostic values |
| `VehicleSelfContactStartup.*` | Bind one TL transaction to the existing owner/publication and declared first profile |
| `RuntimeBudget.*` | Retained, startup and device forecasts, including shared wall/self backing |
| `SelfContactFactories.h` | `SelfContactOnly` for focused runtime qualification and `LoadedWallSelfContact` for the explicit loaded composition |
| `runtime/Prepare.cpp` | Factory allocation and private contribution installation |
| `runtime/Operations.*` | Call TL accepted assembly/candidate sealing and copy authenticated observations |
| `runtime/Stages.*` | Participate in dynamics assembly, candidate validation, scratch publication and discard |
| `SelfContactStageError.h` | Preserve typed stage/status/source diagnostics without borrowing mutable report storage |

`vehicle_run::ContactComposition` selects `LoadedWallSelfContact` only for the
explicit wall+self controller profile. Its source setup and wall setup must
refer to the same physical execution backing. Preflight validates capacities
and phase peaks before preparing retained runtime storage.

## Attempt lifecycle

`Assemble` obtains accepted force/STI data and a TL accepted-assembly receipt.
`SealCandidate` validates the prepared owner/epoch/attempt and continuous
geometry, then replaces that authority with a completed transaction receipt.
The contribution exposes its scratch receipt to the common physical publisher.
`Discard` clears local receipts and invokes TL trial discard. Only the existing
common dynamics commit makes the attempt's observations accepted.

TL owns CUDA broadphase, feature identity/ownership, force/STI, continuous
certificates, deterministic reductions, and transaction limits. Host feature
and continuous-geometry work remain part of that TL pipeline. Resource guards
are external measured limits; increasing a guard allowance does not increase
the module's native capacity or change its physics.

## Profile and qualification boundary

The first profile is frictionless centered Q4/T3 shell contact at facet level 0.
Original friction, damping and soft-card values are retained as provenance.
Solid/beam contact primitives, exact bilinear Q4 and persistent friction history
are separate future capabilities. The wall+self controller profile requires
the selected V5 assembly and exactly 200 ns physical steps.

Historical M2 qualified a sealed/discarded self-only attempt and one combined
wall+self commit. This does not establish current repeated controller
acceptance. The pending gate is two actual commits, authenticated archive
closure and exact Chrono replay. Current numerical source pins, frozen
fixtures, resource policy and pause/resume status are in the
[workspace plan](../../../planning/CURRENT_EXECUTION_PLAN.md).

## Owning checks

The standalone CMake project exposes `vehicle_self_contact_values`,
`vehicle_self_contact_runtime_values`, `vehicle_self_contact_runtime_coupon`,
and `vehicle_self_contact_runtime_source_proof`. Real-source geometry and
runtime targets are explicit opt-ins with their required fixture/CUDA inputs.
See [RuntimeSourceProof.md](tests/RuntimeSourceProof.md) for the source proof's
scope; textual wiring checks do not replace numerical tests.

Use TL's small unit/frozen-replay and deterministic CUDA checks when contact
algorithms change. Use original-source coupons before the expensive full V5
gate. Do not rerun a full vehicle traversal for documentation or the unrelated
host viewer hash utility. GPU/heavy execution is currently paused.
