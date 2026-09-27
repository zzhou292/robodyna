# Ordered connector diagnostic operands

This isolated successor from TL 90a4a806 moves TYPE13 and TYPE25 independent
endpoint arithmetic from one-thread finalizers into bounded parallel staging.
It reuses the existing shell field reads, means and dot products and one shared
native RHS work helper. It preserves six TYPE13/four TYPE25 work channels, every
per-endpoint +=, native signs, kick_dt, angular fixed_dt and final finite checks.
No tree reduction, atomics, softened law or tolerance change is introduced.

Each candidate follows evaluation, staging, then the original finalizer. The
complete source-order element-status scan still precedes all diagnostic work.
A later element error therefore beats an earlier diagnostic overflow. Failed
rows overwrite their packet with zero without reading unavailable trial/model/
accepted/nodal data. Derived nonfinite operands are deliberately staged rather
than rejected early. TYPE13 retains fmin; TYPE25 retains ordered less-than.

A private packet is 144 bytes per TYPE13 parent or 112 per TYPE25 parent. The
existing bounded arena, startup upload and host forecast charge the complete
region. No per-step allocation or host transfer is added. Direct Measure bodies
are untouched and remain available. Source/CUDA admission and the one owner,
clock, accepted history, publication and discard rules are unchanged.

The tests authenticate full frozen Measure and Candidate bodies from 90a.
Generated host/device comparator functions contain the literal frozen/current
finalizer bodies; the generator changes only declaration/namespace adaptation.
They check all scalar/control fields by value and all double fields by bits,
including signed zero, cancellation, overflow, failed rows and failure priority.
They do not memcmp independent struct padding. Device coupons vary scheduling
and repeat private staging, and the budget tests compare current/frozen arenas.
These coupons are not a full vehicle or public-owner qualification.

Before integration, root must build both actual production batch targets, run
existing TYPE13/TYPE25 native/public-owner suites (including mapped paths and
rollback), inspect real resource forecasts and compare the same complete vehicle
cadence saved states and uninstrumented PrepareStep time. Parallel leaf staging
may reduce finalizer time but no speedup is claimed before measurement.

The independent assembly successor adds distinct `assembly` arena regions and
optional mapped arguments. This branch adds only `measurement` regions, leaving
those arguments untouched. Merge both additive layouts deliberately and rerun
exact current forecasts. No active source or binary is replaced by this draft.
