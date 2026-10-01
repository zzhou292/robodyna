# Mesh rename compatibility baseline

These tests deliberately use the inherited mesh names before the canonical
implementation moves. The future legacy aliases must keep them usable afterward.
They currently link the native aggregate; this is not independent FE/MBD linkage.

`//tests/mesh_compat:baseline_tests` covers fixed/active position and velocity
offsets, one-based node indices, shallow clone topology and surface ownership,
system detachment on clone, the exact historical clear behavior, and initialization
invalidation when adding a node to a live mesh.

The coupled case attaches a mass-1 FE node to a mass-2 rigid body with the existing
node/frame constraint and a stiffness-12 FE spring to a fixed node. A single SMC
system advances 1000 steps of 0.0001 s with MINRES and the existing linearized
implicit integrator. Checks include analytic displacement, attachment constraints,
reaction/acceleration consistency, owner identity and the common time. It requests
one CPU thread, no visualization and no GPU.

Surfaces and nodes are shared by the inherited clone. Shared surface owner pointers
still reference the original mesh. `ClearElements` and `ClearNodes` retain load
surfaces until `ClearMeshSurfaces` is called. These observations are compatibility
requirements for a rename, not claims that cloning creates an independent physical
model. No mesh serialization or restart functionality is introduced here.

The initial tests must pass before applying the mesh rename. An unchanged-source
failure needs investigation and a recorded scope; do not change numerical behavior
or weaken checks merely to complete naming work.
