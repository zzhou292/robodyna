# Optional compact grid inventory

The original axis sweep remains the default. `Limits.strategy=CompactGrid`
selects a generic runtime acceleration with an explicit `max_encounters`
capacity (1 through 32Mi uint32 ordinals). Task and final pair caps are
unchanged. No vehicle IDs, Starter grid parameters or captured state enter it.

## Index and arithmetic contract

Active secondary validation retains the existing order: zero stiffness skips
all remaining fields; finite current position and optional domain screening
precede gap validation. Only admitted positions contribute to index bounds.
The deterministic resolution is the smallest power of two whose cube covers
the active occurrence count, bounded at128. Each nondegenerate axis uses that
resolution; an axis with zero or overflowing finite-endpoint span uses one bin.
No floating cube root or per-model cell length is introduced.

Finite coordinates are clamped to the active bounds before normalized floor
assignment. Strictly interior subtraction/division has nonnegative bounded
operands; results clamp to [0,cells-1]. The x-fastest integer key uses actual
cell strides and is at most128^3-1, exactly represented by existing double key
storage. Main envelope endpoints use the same monotone mapping. Disjoint
envelopes visit no bins; single-bin degenerate axes remain conservative.

Every positive-stiffness main uses the unchanged `ScreenBounds` with maximum
active secondary gap. These bounds contain every row-specific screen box by
monotonicity. Count/fill visits the full intersecting cell rows and applies the
same inclusive three-dimensional envelope membership before packing actual
secondary ordinals. Final strict screen, diagonal test, own-node/removal
filtering, velocity validation, native packing/PEN and ranked pair sort remain
the existing authority. Inactive mains skip geometry as before; no global index
pass reads velocities or rejected-row fields.

## Complete count before publication

Per-main coarse counts and ceil(count/256) task counts receive independent
exclusive scans. The host receives complete counts before any packed ordinal
write. Both encounter and task capacities must pass. A second identical
traversal fills ordinals contiguously by main; original role duplicates remain
distinct. Existing task construction then chunks these compact per-main ranges,
and existing count/scan/fill/sort kernels produce final pairs. Fill/count
inconsistency rejects without publishing a view. `Report.encounters_counted`
and `pairs_counted` explicitly describe which stages completed.

## Allocation and lifetime

Only compact strategy allocates two uint64 arrays of mains+1, one GridControl,
and four bytes per declared encounter. Main bounds are recomputed with the
same immutable query during fill, avoiding a persistent48-byte-per-main array.
Existing CUB scratch already covers the additional mains-sized scan; no new
scratch maximum or task/pair cap is required. The ordinal buffer is work storage
inside each inventory arena, never a physical state or history cache.

The owning leaf publishes `Forecast.index_device_bytes`, already included in
total device bytes; Transaction's existing paired-workspace forecast charges
both copies. For341,504 mains and16Mi entries the proposed incremental payload
is approximately72,573,080 bytes per inventory (including a136-byte control,
before final alignment), or0.13518GiB for the self pair. A32Mi cap would add
0.26018GiB for the pair. Start with16Mi only after exact leaf/app forecasting;
retain legacy wall enumeration unless its own counts require a change. This
is bounded capacity, not a promise that arbitrary input will fit. The existing
6GiB process GPU-growth guard is unchanged.

## Qualification

Independent coupons compare every canonical pair/offset to original native
local reference and legacy inventory over separated/overlapping bands,
degenerate axes, translated boundary neighborhoods, repeated role/source IDs,
source removals, signed coefficients, motion/gaps and Native/SI coordinates.
Zero-stiffness/out-of-domain poisoned fields must remain unread; consumed
nonfinite inputs must still reject. Exact/one-short encounter/task/pair and
byte caps preserve no-view publication and retry. Existing candidate and
inventory suites remain owning regressions. Full owner counts follow only
after source review and the guarded numerical gate.
