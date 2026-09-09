# TB1/TB2 resident triangle batch results

2026-09-09. All eight actual CUDA functions pass for the standalone prescribed
one/two-triangle batch. The 47 existing native/startup/rates/force functions and
41 existing nodal-owner functions also pass. Repeated executions do not add
distinct functions. CMake and the direct Bazel CUDA `elements/t3:batch` archive
build pass. This qualifies resident history, force-cache assembly and publication;
it does not qualify mixed Q4/T3 dynamics, wall impact or source MAT024/NIP3.

The load/hold/reverse/hold test compares twelve native/CUDA element intervals,
including all complete force/history fields and independent work checks.
Prescribed targets, both zero-rate holds, high uint64 source IDs, shared native
mass/inertia, negative internal-force scatter and nonzero-work controls pass.
Foreign/stale views, late second-element failure, invalid launch and rejected
receipts preserve accepted state; a clean retry matches the clean owner's fields.
Nonzero cached forces are checked in discarded assembly trials, not used to
claim a qualified coupled trajectory.

The batch has one **6,216-byte device allocation**, matching the pre-execution
host declaration probe and CUDA ABI assertion. Its existing nodal owner retains
six allocations. The fixture uses one GPU thread, at most two triangles and
sixteen covered owner nodes. These are qualification capacities, not vehicle
throughput measurements. Every heavy operation used the serialized workstation
guard, one build worker and at most two affinity CPUs. The largest sampled RAM
use in the batch build checks was 588,447,744 bytes (direct Bazel build), below
the user's 16 GB ceiling. Sampling can miss short-lived peaks.

The frozen aggregate energy scale is **1e-6 J** and its absolute floor is
**1e-18 J**. No numerical budget, production formula or native port changed
after execution. The initial malformed-reference fixture accidentally duplicated
a local source ID, so it failed inside reference construction before reaching
the intended batch check. The corrected valid reference instead breaks a shared
global-node identity and checks the exact failing element/node. The complete
[first execution snapshot](../../../../../crash-work/checkpoints/t3-batch-first-execution-1/manifest.json)
retains that failure. A separate GTest report property originally selected the
integer overload and displayed the 1e-6 J scale as zero; its source and passing
XML are preserved before changing the property to a decimal string. That report
field is not an input to any numerical comparison.

The initial Bazel invocation rejected an unsupported startup argument before
loading the workspace. Its report remains alongside the subsequent successful
direct CUDA build. This was an invocation error, not a compilation or physics
failure. Reports use the `crash-work/reports/t3-batch-` prefix. Source-map revisions
record the fixture and report corrections; prior checkpoints remain immutable.

The next element milestone is an immutable mixed Q4/T3 mass binding and a
two-participant preflight/commit boundary using the same owner. Family histories
and native angle-weighted triangle inertia remain distinct. The proposed shared
QEPH helper adoption is a separate regression step after this checkpoint.
