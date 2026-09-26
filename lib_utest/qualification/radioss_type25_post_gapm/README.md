# Mixed post-GAPM topology and current normals

This opt-in value profile consumes genuine mixed SH2 sides plus an explicit,
source-resolved post-I25GAPM support/permutation descriptor. It does not resolve
physical support, calculate coefficients/gaps, disable failure, or admit runtime
publication. Existing ordinary/resolved-shell APIs and defaults stay unchanged.

The caller supplies P primary faces, S shell faces and G=P+S; only shells have
partners. Contact primary/global identity, all raw face origins, and mechanical
support identity remain distinct. Every raw origin must belong to exactly one
primary. Multiple-origin primary source_id0 is accepted only with the complete
explicit discriminator/count/origin proof. A primary reversal applies after
SH2 and never regenerates its partner.

Pre-shell internal count is retained independently from final shell overrides.
IDEL is the supplied incoming flag unless that pre-shell count is zero. Internal
segments are excluded from ordinary incidence and Starter normals. For IDEL1,
complete solid-support incidence supplies the additional native neighbor lists.
Indexed edge buckets preserve candidate ID order; each main's four lists retain
cross-edge ADD_ID deduplication. Reference unions run ordinary then additional
neighbors, in native edge order. The edge, tag and candidate buffers are reused
only after their earlier phase is dead; no dense per-main neighbor matrix or
per-edge scan over all G is introduced.

The host current-normal overload authenticates full immutable source metadata
and exact origin counts. MixedSourceValidationBytes(P) reports its separately
bounded temporary counter allocation. Production CUDA numerical helpers receive
only partner[P] and existing P/G/roles/CSR; no host Snapshot pointer or provenance
upload is required. Runtime owns source authentication, accepted/trial cache,
activity, stream scheduling and publication. Old public numerical APIs reject
the mixed profile.

Qualification uses the complete pinned SH2, I25NEIGH (including IDEL1), native
CSR, I25NORM, NORMP1/2 and FREE_BOUND from the existing startup reference target.
The new wrapper only supplies source-shaped post-GAPM operands and reads results.
Its private IELEM packing preserves the solid/shell partition and nonzero second
support predicates consumed by these routines; it makes no native vehicle row
order claim. Native current recurrence uses independent prior caches. All native
wrappers share COMMON and must execute serially.

Fixtures start with genuine source extraction and IN24/filter/SH2. They cover
solid-only and mixed Q4/T3/coated faces, internal interfaces, multiple exterior
edge candidates with IDEL enabled, explicit source-phase permutations, negative
and zero coefficients, inactive cache, aliases, exact host cap and failed retry.
Permutation coupons prescribe a valid upstream operand; they do not claim to
reimplement the separate source support/geometry authority. GPU tests compare
full native intermediate and final float fields across launch widths/order.
