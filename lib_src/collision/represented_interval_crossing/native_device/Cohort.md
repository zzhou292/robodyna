# Optional numerical-cohort cache

This slice is source-authored and unqualified until its owning receipts pass.
`numeric_cohort_pairs=0` keeps the qualified per-slice GPU path. An explicit
capacity of at most 4096 enables lookahead only for native compound calls. The
default 128 workers, per-pair depth/work and original native input/result/work
limits do not change. No vehicle execution path is selected by this module.

## Authority and ordering

The existing outer helper checks all canonical pair rows and caller ranges.
The native owner authenticates every path, including unused paths and shared
trajectories. Only then may its lexical scene create a noncopyable numerical
cohort. Both scene and cohort use private user-provided construction keys; actual
C++17 negative-compilation tests reject forged braced keys.

Lookahead additionally requires the complete input ranges to be mutually
disjoint and outside native-owned storage. This is derived privately using the
same native range checks. If the stronger proof is unavailable, keep ordinary
per-slice GPU execution and its original alias/failure position. A pure range
coupon models an unavailable full-range proof with safe typed arrays; the source
gate binds that result to legacy selection. No unchecked caller flag or unsafe
type-punning qualification hook is introduced.

Nonfinal numerical windows contain whole original publication slices. The final
window may be shorter. Consequently no row is prefetched twice, and a window
smaller than an admitted slice rejects explicitly before launching work.
CanonicalPair.h shares the unchanged source-key normalization for ordinary
slices and lookahead. It creates values, not source authority.

The workspace applies the same actual-coordinate domain test to each window.
Eligible rows run in the unchanged CUDA kernel; other rows retain the original
CPU execution when their publication slice arrives. Returned values enter a
bounded host cache. The workspace retains no scene/cohort pointer or permission.
Readiness belongs only to the native lexical cohort and expires with it.

Before consuming a slice, compare every cached complete pair key against the
ordinary native canonical rows. Copy only that slice's GPU rows to staging and
leave CPU rows incomplete for the original persistent workers. The unchanged
native canonical fold then applies that slice's exact work limit and publication.
A later work rejection cannot publish prefetched later rows. Numerical
Unresolved/ExactArithmeticRange remains a typed completed outcome at its original
work, never CUDA failure, separation or automatic CPU retry.

## CUDA failures and diagnostics

Transport.cpp drains every queued borrowed read, preserves the first CUDA error,
and poisons the owner on CUDA or device-domain disagreement. A failed larger
launch has no successful sub-slice publication to claim; retain the last complete
publication before that numerical window. Fault metadata records window bounds
and a full source ordinal when known. A future ordinal is not misreported as a
failure of the current native slice. No GPU failure retries on CPU.

`device_pairs` counts admitted numerical jobs; failed transfers may prevent their
execution. `consumed_device_pairs` counts rows copied into native staging, which
can itself later fail the native work check. Neither is a physical commit count.
`numeric_cohorts` and actual launch/upload counts distinguish lookahead from
consumption. The benchmark must retain these distinctions on failures.

## Allocation and qualification

Only opt-in mode allocates cached result/route rows. Device and host job/result
capacity is the larger of existing native result capacity and the chosen cohort
capacity. Original native capacities and their CPU pool remain unchanged. Every
aligned byte is admitted in startup forecast; no query grows storage. Worker
scratch/DFS and compiler device-local memory retain their existing bounds.

Owning tests compare CPU, legacy GPU and cached GPU full batch reports, outer
rows, path-roster work and last native publication. Cases cover 257/4097 boundaries,
nonmultiple capacity alignment, all-wide and initially-wide windows, typed
unresolved results, native late work rejection with unconsumed prefetch, changed
same-address input, malformed unused paths, aliases, empty calls, exact/short
caps, and injected copy faults in the first and a later numerical window.

The separate compound benchmark must include source upload, pair transfers,
kernel execution, CPU fallback, synchronization and native folding. Compare the
same binary's CPU compound, legacy 256-slice GPU and opt-in numerical-cohort GPU.
The mixed and eligible-only diagnostic views remain distinct; no production
contacts are omitted. Promotion requires measured benefit plus owning gates.
