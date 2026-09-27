# Ordered mapped connector assembly

Implementation plan (2026-09-26, before source changes)

TYPE13 and mapped TYPE25 currently scatter every connection in one GPU thread.
Replace only mapped assembly with one writer per physical node. Reuse the
existing mapped-shell incidence builder and staged node/publish representation.
Build source-parent/local-endpoint incidence once in the existing bounded arena.
Only touched nodes receive private sums, bounded by the smaller of the full
node count and twice the connection count; untouched-node bits stay unchanged.
The complete startup, steady device and host forecasts include every new byte.
Unmapped paths retain their original implementation.

The stream executes five bounded phases: initialize identity/control, prepare
independent parent operands, gather each node in original incidence order,
select the first failure, and publish only on success. Forces keep sign +1 and
each channel's exact addition order. Existing family coefficient, accepted-node,
constraint, mass/inertia and initial-state checks remain authoritative. Only
integer diagnostic minimum keys are atomic. No force or stiffness atomics, new
clock, reordered floating reduction or model-specific branch is introduced.

TYPE13 validates all endpoints before any assembly. Its failure keys therefore
rank every endpoint failure ahead of any coefficient/addition failure. TYPE25
validates and assembles in parent order; its keys rank parent preparation before
that same parent's addition. For either family the first offending endpoint
within one parent remains source slot order. A failed mapped assembly publishes
no staged destination; the existing owner still rejects/discards the attempt.
This strengthens private destination preservation without changing accepted state.

Tests will compare complete force, couple and STI/STIR arrays against the old
serial scatter bit for bit, with shared nodes, unequal initial sums, cancellation,
zero coefficients after failure, untouched nodes and shuffled execution launch
shapes. Validate competing failures and each family's distinct ordering, invalid
constraints, overflow, repeat/reset behavior, bounded layout exact/short caps,
legacy regressions and mapped owner discard/retry. Qualification will use focused
host/CUDA suites and the existing owner gates; root owns builds and GPU jobs.

File ownership: assembly helpers and mapped arena/startup/forecast wiring only.
Candidate/Finalize numerical diagnostics remain a separate work lane.

## Actual owner storage and rejected scratch contract

This schedule is private to authenticated mapped entrypoints, not a replacement
for the low-level scatter API on arbitrary borrowed memory. Both mapped
AssembleMappedAccepted methods call shell_physical_owner::BorrowAssembly before
any assembly kernel. That calls FENodalState::AuthenticateAssemblyView, whose
SameAssembly comparison checks every force/couple pointer, count, source pointer,
stream and attempt against the owner's exact live capability.

FENodalState::Impl::ActiveAssemblyView constructs six distinct n-double spans in
its own cudaMalloc scratch allocation. BorrowCinAssembly supplies STIFN and
STIFR as two distinct n-double spans in the separately allocated CIN arena.
Shifted force buffers and force/STI cross-aliases therefore fail existing owner
authentication before dispatch. The actual TYPE13 and TYPE25 mapped-owner tests
assert all eight physical ranges are disjoint and exercise those forgeries,
accepted-state preservation and a successful fresh-token retry.

FENodalState.h declares forces and bounds to be trial SCRATCH, never accepted
force diagnostics, and requires discard on recoverable failure. NodalForceAssembly.h
requires coordinator discard after any contribution failure. The mapped TYPE25
contract similarly requires all rejected contributions to be discarded. This
optimization keeps the same first failure but preserves every incoming scratch
value on rejection; it does not promise the old partially scattered private RHS.
Accepted forces, histories and publication remain unchanged, and existing owner
rollback tests remain mandatory. The frozen serial comparison covers complete
success outputs and failure diagnostics, not rejected scratch equality.
