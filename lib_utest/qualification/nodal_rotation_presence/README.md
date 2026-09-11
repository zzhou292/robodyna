# Rotation-free ordinary nodes

The existing `FENodalState` supports an explicit immutable per-node rotational
DOF presence mask. A solid-only ordinary node has translation, zero inverse
rotational inertia and zero spin. It keeps its orientation and has no rotational
reaction. This differs from a fixed rotation, whose applied couple produces a
reaction. A nonzero or nonfinite couple on an absent rotation rejects the whole
trial; loads are never silently discarded.

Null presence preserves the existing all-present rotational owner. The optional
mask occupies one byte per node in the existing constraint allocation. It adds
no allocations, solver, history selector or clock. Actual accepted assembly
identity includes the mask, and nodal stamps record its presence.

Five CUDA functions qualify analytic half-kick translation, actual rotation and
fixed reaction in one owner, immutable startup requirements, exact byte budget,
late-couple rejection/retry, coexistence with two ordinary rigid groups, and
three ordered CIN intervals plus accepted/prepared zero-J readback. The latter
retains real positive-inertia CIN masters/dependents and adds an unrelated
solid-only node. Existing rotation, temporal, history-admission, CIN owner and
rigid startup tests are linked into a separate regression executable.

Root gate `nodal-rotation-presence-root-tests-2` passes all five new functions
and the separate legacy regression executable. Attempt1 exposed the existing
CIN snapshot validator still reconstructing `1/J` for a rotation-free node;
that validator now checks explicit absence and rejects any nonzero raw J there.
No integration equations or test tolerances changed to close that failure.

This increment does not admit zero-J rigid-PART members or mapped shell/solid
contributors. Rigid members and all CIN masters/dependents must retain
kinematically present rotations. Actual PART runtime admission and full ledger
composition remain separate integration work.
