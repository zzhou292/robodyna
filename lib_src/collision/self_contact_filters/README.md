# Exact endpoint-hull coincidence

This private helper accelerates `CertifiedLinearFacetPrismSeparation` without
changing what it certifies. The public header and existing input validation,
axis order, directed interval arithmetic and thickness inflation are unchanged.
All geometry, both positive finite thicknesses, the axis-limit enum and output
validity pointer are handled before the new check. A null validity pointer still
returns before initializing the axis output.

The original certificate projects every vertex in each accepted/prepared endpoint
hull. If any represented vertex position in the first hull numerically equals one
in the second, every finite candidate axis gives the same point projection on both
sides. Their projected hull intervals overlap at that value, and positive thickness
inflation cannot separate them. Signed zeros represent the same value. Overflowed
axis/projection arithmetic was already unresolved. Thus none of the original axes
can return strict separation; the exact old result is false, valid=true, axis=None.
At most 36 fixed three-dimensional point comparisons prove this necessary-no-separator fact.

All four endpoint combinations count, including first accepted versus second
prepared. Cross-time hull coincidence does not imply a physical intersection:
two separated triangles under common translation can have overlapping swept hulls.
No source key, model ID, topology classification, ownership rule, tolerance,
geometry perturbation or accepted-force shortcut enters this check. The pair still
passes through the unchanged discovery, continuous geometry and contact policy.
The accepted static filter likewise retains ExactRemaining for a coincident pair.

One private compile-time implementation supplies production and reference
qualification. Production fixes the early check on and observation off. Private
`PrismQualification.h` exposes only value comparisons of the original/current
result tuple (separated, valid, axis) and bounded operation counts. No runtime
profile, global switch, per-pair clock or physical proof-work change is introduced.
IEEE sticky flags and errno from skipped arithmetic are not claimed identical;
as with existing proof-order optimizations, they are not solver acceptance inputs.

Eight owning host groups cover all axis limits, both endpoint/cross-time orders,
signed zeros and unrelated source keys, adjacent representable noncoincidences,
invalid other vertices/thickness/enums, null outputs and extreme finite overflow.
A 288-pair signed-coordinate/vertex-order family reuses the pinned failure 8 Yaris
coordinate bits and its original 0.0005 m half-thickness, with generic source IDs.
It compares complete result tuples and counts 148 original axis attempts versus
zero current attempts. These are bounded coupon operation counts, not a measured
vehicle hit rate or elapsed-time speedup. Existing medium/full transaction checks
remain necessary. Source is isolated from the active vehicle run and unbuilt at
this authoring checkpoint.

Independent source review found no blocker at the 2026-09-24 checkpoint. All
new groups remain unbuilt/unexecuted; qualification and actual runtime/hit-rate
measurement are pending. No active-run source or physical checkpoint was changed.
