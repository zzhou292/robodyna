# Explicit extended solid coefficient order

`InitializeWithExtendedSolids` creates the V4 ledger from an immutable
`ExtendedLaw44Law90` snapshot. It preserves the existing shell, TYPE25, TYPE13,
point-mass and three-solid order, then appends rear LAW44 and radiator LAW90
parents in their supplied order. Every original slot contributes exactly once,
including repeated H8 slots at the same physical node. New named subtotals and
occurrence counts keep each family's contribution inspectable. These solids
add no rotational inertia.

The legacy ledger entries still reject the extended snapshot; V4 rejects a
legacy or missing snapshot. Versioned identity includes exact retained source
identity, domains and original mass values. Existing bounded arena accounting
uses the enlarged actual node/implementation types; caps are unchanged.
The snapshot's source receipt changes only for the reviewed ledger admission
check and the additive shared test fixture export, retaining prior file records.

Four new host functions cover original-slot/node-order sums, independent nodal
mass identities, repeated slots, exact zero added inertia, complete diagnostic
counts, borrowed-source lifetime, both new structural collision roles, wrong
domain, versioned matching, unchanged failed initialization and exact cap/retry.
The nine snapshot and sixteen previous ledger functions run alongside them.

Root qualification: `extended-solid-ledger-tests-1` passes all29 functions and
the snapshot/source identity (0.24s CTest). A separate author reviewed the
production diff and admission/order/identity/cap tests with no blocker found.
This is coefficient assembly only; the model, force state and physical owner
must independently admit and execute the new mechanics families.
