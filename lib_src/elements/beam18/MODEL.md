# Immutable beam18 source model

`beam18::Model` is a source-order collection of qualified Circular4 stored-zero
LAW44 beam references and prepared materials. `Initialize(domain, input, limits)`
retains the immutable canonical domain and owns complete curve copies, pooled by
original MID in first-parent order. Repeated MIDs require bit-identical named
material scalars, derived preparation values and both complete curve arrays.
There is no caller-owned curve lifetime after successful initialization.

Every parent retains its complete original Reference, including EID/PID/SID/MID,
original working-unit coordinates and orientation N3. Its two `domain_nodes`
are N1/N2 only. Endpoint coordinates must match the canonical SI bits exactly.
N3 may be absent from the domain; when present its source coordinate converted
once to SI must match. A present N3 gains no endpoint coefficient from this beam.
Coincident distinct NIDs are retained; zero-length endpoints are already rejected
by the qualified reference constructor. A successful Model does not advance or
create force/history, allocate CUDA memory, admit DOFs, or define a clock.

The model's `Endpoint(parent, local, out)` copies the qualified PMASS endpoint
mass and **total native scalar inertia**, plus startup STI/STIR/STP observations.
No physical/added inertia decomposition is invented. Working-unit coefficients
retain the original reference's conversion order. Queries preserve output on
invalid parent/local; source/material views are immutable.

`Beam18NodeContributions::Initialize(model)` retains this model and its domain,
then snapshots exactly two endpoint records per parent. It has no separate domain
copy or model cycle. Records are model-parent order then endpoint 0/1.

`NodalCoefficientLedger::InitializeWithBeams` is the explicit V5 order:
QEPH, T3, QBAT, TYPE25, TYPE13, ELEMENT_MASS, solid18, solid24, S6Z,
solid18 LAW44, solid18 LAW90, beam18. The beam snapshot is required. Its physical
prefix may omit solids or contain either existing typed solid snapshot; the
actual full-case profile separately requires its complete five-family model.
Existing V1-V4 initializers and matches do not admit a beam source implicitly.
Structural beam EIDs share the Q/T/B/TYPE13/solid namespace; TYPE25 WID and
ELEMENT_MASS source IDs remain independent. Every beam term adds its exact M/J
to the current authoritative node sum, once, after the prior families. Diagnostic
beam subtotals and occurrence counts are separate from authoritative totals.
Uncovered domain nodes remain explicit zeros, without any DOF admission claim.

Count and byte caps precede allocations and borrowed array traversal. The model
charges its arena, retained domain, implementation/control allowance and two
bounded transient source indices. Snapshot and ledger forecasts retain the
whole producer graph; only authenticated identical domain backing is discounted.
`owned_payload_bytes` describes retained payload; `startup_payload_bytes` adds
construction scratch. These are bounded payload reservations, not process RSS.
Failed constructors leave their destination empty and support retry. Copies
share immutable backing. Shared physical output guards include model/parent,
material/curve, contribution and domain ranges.

The downstream solid participant must explicitly accept V5 while matching its
own retained snapshot. The common publisher must require the actual beam
participant and exact Model match whenever the ledger contains beam18. Neither
of those runtime admissions is implemented by this model/ledger change.
