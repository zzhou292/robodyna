# Reuse the prepared intersection order in residual checks

This private optimization changes only the intersection-presence lookup inside
LinearResidualForTasks. The transaction already constructs SortedIntersections
after successful native discovery and prepared-intersection validation. The two
main linear/quadratic residual calls borrow it synchronously in that same cohort;
no later Discover or buffer mutation occurs before those calls return. Nothing
is retained across attempts or accessed after discard.

All original triangle-key/view/thickness validation and reference-translation
calculation remain before lookup. Finding any matching canonical pair returns
the same PotentialContact result with the same fields. Both paths compare the
full pair key: source instance, parent element, level and local facet, with the
same producer-order normalization. Intersection kind and local-exclusion metadata
are not used as permission. This borrow proves ordering only; it adds no contact,
ownership, continuous-motion or force certificate.

Raw APIs retain the original full-scan loop. Indexed overloads use the existing
SortedIntersections::Find; a foreign, unsorted, duplicate or otherwise unmatched
view falls back to the existing canonical first-match scan. Following residual
bounds, feature scanning, masked fallback replay, curvature handling and every
error/status/work rule are unchanged. Feature dedup/indexing is outside scope.

No new retained buffer, allocation, thread or forecast charge is introduced.
For a matching immutable publication, repeated lookup changes from O(I) to
O(log I), reusing the ordering check the candidate path already paid. This is an
operation-count argument, not a measured whole-stage speedup.

Qualification compares every residual field (including floating bits) between
raw and indexed calls for present/absent keys, all key fields, reversed producer
orientation, malformed and mismatched views, validation-priority cases and real
linear/quadratic geometry. Existing sorted-lookup operation-count coupons remain
the lookup oracle. Coupled candidate/publication/budget/rollback tests and frozen
policy replays must retain exact outcomes, work and semantic digest. The source
is isolated from the current vehicle executable until owning gates pass.
