# Native TYPE25 selection lifecycle

This stage composes the selected local native lifecycle through **selected raw geometry**. It is a numerical adapter, not a physical contact owner or a complete vehicle profile. Runtime source authentication, complete search-inventory binding, current-state/attempt identity and common physical publication remain coordinator responsibilities.

The order is Begin/I25IRTLM; raw OPTCD filtering and persistent ICONT_I transition; deleted-main release; retained classification; three/four-slot PREP_SLID1; original adjacency/dedup PREP_SLID2; complete continuation membership snapshot; complete new-impact membership snapshot; MAINF history reset; KEEPF; then final-cache selected geometry. Post-force marker normalization is a separate explicit operation called after offset/history, response and assembly, before commit. A successful selection call does not complete those later operations.

`RadiossType25Lifecycle.h` exposes the shared typed source and private row-operation implementation. Nodes contain immutable IDs/constraints only. Positions and velocities are borrowed views of the sole owner's accepted X_n and native velocity stage. Native working units are retained for geometry/history/coefficient/TT/DT1; optional SI X/V are converted per packet without copying the physical model. Raw IRESP must be explicit: 0 and 2 choose the source non-1 branch; 1 chooses the special precision floor; unknown -1 rejects. ICONT_I is incoming persisted runtime state and must be replaced by each successful RowResult.initial_contact_flag, never reconstructed from Starter each step.

The shared GPU seam is Validate → Requirements → PrepareRow → global complete-count scan/admission → CompleteRow. Requirements bounds only sliding scratch; PrepareRow retains begun/classified row and cache, saved OPTCD markers, actual sliding main list, and exact required CAND_OPT count. CompleteRow creates numerical outputs only after that count is admitted. One thread owns a secondary row; the coordinator must provide disjoint private scratch and immutable borrowed sources until all kernels complete. This module does not supply an unchecked physical executor or create a new clock.

Raw search occurrences are two native integer identities (8 bytes); they carry no fabricated classification cache. Complete local phase coverage proves each admitted retained/optimized/sliding occurrence gets exactly one cache-producing classifier. Missing cache coverage is an error. Final global order is retained rows in secondary order, admitted raw original ordinals, then secondary/slot/adjacency append order. Per-row CUDA scheduling changes none of those ranks. Cache occurrence IDs must be rebound to this final dense order before ASS0. Signed candidate exclusion and opposite-side main rewrites are preserved.

The bounded host convenience adapter uses conservative scratch for its supplied small corpus, then atomically replaces the caller's result after all rows succeed. Its allocations are checked against Limits using sizeof of actual types and checked arithmetic. The runtime uses the compact exact-count GPU seam instead of allocating caches per raw candidate. Late numerical failures retain the globally complete required count and the actual row/stage culprit. Original native small-cap OPTCD may write a partial private prefix; our public adapter deliberately publishes nothing on resource failure. Successful native numerical behavior is the parity contract, not that unsafe partial-write failure behavior.

The independent native oracle executes full pinned OPTCD/PREP1/PREP2/KEEPF and original membership/clear/release/post-force blocks, composed with the already qualified independent Fortran Begin/End/classification/coefficient/geometry references. It calls no production numerical operation. Generated modules are private to avoid symbol collisions with parent oracles. See native/README.md and source-manifest.json. The oracle has explicit small table bounds; these are qualification bounds only.

Host and actual CUDA groups compare every defined cache, row/history/ICONT, selected descriptor and raw-geometry field. GPU results are exact-bit compared with host shared arithmetic; native real values retain the existing 64-epsilon scale-relative comparison and exact integer/mask identity. Tests cover cold/retained/lost/sliding, duplicates, side rewrite, mixed T3/Q4 and authentic omission of duplicate T3slot4 from adjacency, strict precision branches, deleted-main delay, exclusions, underflow, unbound native scratch, two-step ICONT loss/reacquisition/discard/retry, global origin order, exact capacity and late-failure complete count. CUDA readback reorders only integer ordinals; it does not recompute numerical values on CPU.

Still outside admission: foreign/MPI rows, thermal/adhesive/radiation/gap-load branches, negative secondary stiffness preprocessing, arbitrary source completeness, force/history native NVSIZ packet scheduling, and the documented raw-geometry undefined-XP domain. The latter is rejected by production and explicitly rejected by the oracle; no invented point or silent clamp repairs the native donor. Defined-domain qualification does not prove full vehicle pipeline reachability closure.

Configure this directory with TYPE25_LIFECYCLE_CUDA=ON and the pinned GNU Fortran/NVIDIA compilers. It includes all parent normal/friction/geometry/selection/coefficient regression targets. Run host and CUDA CTests separately under the existing workstation/GPU guards. Bazel consumer/source targets own header usability and pinned extraction; native numerical Fortran/CUDA qualification is CMake-owned. No test executable is linked into a product target.

## Optional current-normal view

The host gate adds seven focused groups for the complete optional view, exact
legacy/copied-static/native comparisons, primary/opposite/continuation/geometry
field binding, independent history recurrences, malformed and aliased spans,
unbound NaN scratch, T3 signed-zero slots and the explicit normal-read barrier.
An O0 production-only consumer proves the startup/lifecycle reference aliases
are the same type and require no native backend or physical owner.

`type25_lifecycle_normal_view_cuda` is a separate executable sharing the existing
bounded fixture; six groups compare complete native rows with different launch
orders, optional view layouts, repeated same-address changes and rejection/retry.
The live writable-alias coupon uses the actual `Reference.boundary` int member.
No unrelated-struct cast is used to bind normal fields or create that alias.

Copied Main records occur only in the independent legacy/native test control.
Production selects the borrowed staged arrays through one complete descriptor.
These tests do not implement normal production, synchronize an external stream,
or grant physical publication authority. The immutable source/force-base phase
must be bound by the moving transaction owner after the actual normal barrier.
