# Generic visual models

This owner contains the inherited camera, ordinary visual shape and visual model
implementations. It depends on neutral geometry, materials and the core foundation.
It contains no renderer, physical system, body, FE mesh or element implementation.

`ChObj` remains in `//src/mechanics/object:object`: its existing identity, timestamp,
camera and visual-instance semantics are preserved. This is an intermediate
ownership split, not complete FEA/MBD independence or a renamed visual API.

FEA stays enabled in the shared configuration. A model's private FE updater starts
null and is installed only when an FE shape is attached. The FE attachment adapter
lives in `src/fea/visualization/LegacyVisualAdapter.cpp` and is compiled by the
combined mechanics backend until the concrete FEA implementation is separated.
It runs the existing nonvirtual FE update after all ordinary shape updates.

Copy/assignment retain the updater and shared FE objects; clearing removes both
shape lists and the updater. Sharing only a model does not change an FE visual's
stored owner. Explicit FE reattachment does. Archives continue to store ordinary
shapes only; this separation adds no FE serialization or physical restart support.

Qualification targets (execute under the workspace's bounded test procedure):

```text
//src/mechanics/object:dependency_boundary_test
//src/mechanics/object:object_test
//tests/visual_model:compatibility_test
```

The dependency gate inspects actual compile/header/link ownership. The standalone
runtime test requires FEA enabled and links without the aggregate or FE code.
The existing FE behavior baseline remains an aggregate test; do not substitute
the standalone ordinary-shape test for that coverage.
