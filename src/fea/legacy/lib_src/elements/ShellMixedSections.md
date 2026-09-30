# Joined layered LAW1 and LAW44 sections

Prepare one complete `ShellBatchSectionBinding` with `InitializeSections` or
the explicitly bounded `InitializeSectionCatalog`. Pass that catalog to each
existing `InitializeJoined` family overload. Every native parent retains its
original family index and complete source assignment. The catalog's law tag
selects the already qualified layered LAW1 or layered LAW44 adapter, including
analytic and tabulated LAW44 hardening. LAW1 uses NIP3 and evolving physical
thickness; it has no plastic strain, yield or rate history.

The old integrated LAW1 path without a catalog and the old LAW44 single-material
and catalog paths keep their device layout, allocations and candidate arithmetic.
Only the explicit heterogeneous mode installs the separate mixed arena. Common
publication still compares the complete owned catalogs and source inventory;
different family subsets or a changed other-family material cannot grant scope.
Native nodal M/J, owner stepping and shell force/work reduction are unchanged.

## Storage and resource limits

The optional arena contains one immutable law array, direct per-parent LAW44 and
LAW1 parameter arrays, two pairs of typed section histories, and the complete
owned curve pool. All arrays use the same native family index. Both histories
use the shell participant's existing accepted-slab index; the storage classes
have no clock, selector, pending state or commit operation. Candidate work writes
only that parent's trial values. Initialization owns all curves and rebases only
actual table pointers; elastic and analytic declarations keep null curve views.

On the qualified binary64 ABI, legacy plastic storage costs 640 bytes per parent;
the mixed arrays cost 937 bytes per parent plus headers/alignment/curve pool.
For 328344 QEPH and 21301 T3 parents, the increment is about 104 MB across both
families. This intentionally trades unused private typed slots for a single,
readable indexing rule. The admitted source counts fit the existing 2 GiB device
budget per family; the hard 524288-parent ceiling does **not** promise that every
composition fits. Whole family plus section layouts are checked together before
allocation, and the 2 GiB cap is unchanged.

The forecast includes host facade/arena objects, initialization arena, all three
startup-sized readback arrays, temporary curve offsets and retained catalog.
Vehicle accounting preserves the checked shared-inventory discount. Legacy
device storage is unchanged; the host facade adds one optional pointer (8 bytes
on this ABI), charged through its existing `sizeof` budget. No per-step allocation
or GPU synchronization is added to the disabled paths.

## Authenticated typed readback

`CopyAcceptedLayeredSectionHistory(stamp, output, capacity, diagnostics)` and
`CopyPreparedLayeredSectionHistory(receipt, output, capacity)` return one
`ShellBatchLayeredSection` per native parent. Its `law()` and nullable `elastic()`
or `plastic()` accessors define availability. The legacy plastic-history APIs
reject explicit heterogeneous mode; they never fill elastic PLA fields with zero.

The APIs check actual batch identity, complete counts, all output/input overlaps
and CUDA status before publishing. Both arrays are staged and every active value
is checked for finiteness before any caller output changes. A nonfinite prepared
readback discards that participant's candidate; device failures poison it through
the existing runtime policy. Caller outputs and receipts remain unchanged on
failure. New attempts, discard and common commit invalidate earlier receipts.
Active fields alone define values; private union/padding bytes are not a binary
format, hash or cross-owner equality contract. Serialize through typed accessors.

LAW44 cumulative plastic work remains a separate observation channel already
contained in native shell work, never an additional internal-energy term. This
extension makes no global crash-energy accuracy claim and adds no glass,
failure, membrane or rigid-material approximation.

## Qualification owner

`qualification/mixed_layered_resident` supplies three host layout/availability
tests and six actual CUDA integration tests. The CUDA fixture has elastic and
plastic parents in opposite orders within each family, uses the actual joined
owner/publication transaction through load/hold/reversal, and compares its results
against direct, independently qualified host adapters with carried histories.
It checks yielding/rate/physical thickness/work, source lifetime, complete scope,
stable allocations, late family failure and exact active-field retry. Readback
gates cover wrong/stale receipts, aliases, capacity, unavailable legacy fields,
late active NaN and a completed final copy followed by an injected CUDA error.
The copy hook exists only in the linked test binary; production has no test hook.

Those are dispatch/storage/transaction gates, not another native formulation
oracle or a full-vehicle simulation. Existing native point, layered force and
legacy resident tests remain their owning numerical regressions.
