# Native contact activity selectors

This is a common-publication component, not a removal-policy or contact-force
implementation. The private source attachment may opt into activity generation
one; the old immutable-source path keeps all activity fields zero.

An accepted-force stage uses the accepted activity generation. Its search
reference records that generation independently of the next prepared source.
A candidate may retain the accepted activity slab or stage the opposite slab
with exactly one new generation. If activity changed, the next force stage must
rebuild a search reference for it; an old reference cannot be relabeled current.
All device activity/source validation remains the native transaction's job.

Common publication still consists only of infallible selector stores. Discard
cannot expose staged activity, history or search selectors. The actual owner
CUDA tests cover next-activity/force-base distinction, mandatory search refresh,
late rejection and retry, invalid generation/slab plans and unchanged legacy
admission. The existing physical owner fixture is shared, not reimplemented.

No nodal state, clock, force, mass, geometry or material law is added here.
