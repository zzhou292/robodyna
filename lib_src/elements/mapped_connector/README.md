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
