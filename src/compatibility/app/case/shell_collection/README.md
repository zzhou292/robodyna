# Shell collection contact geometry

`ShellCollectionContactGeometry` adapts a complete immutable TL shell binding
to the existing reference-area nodal contact model. It reuses
`Q4ParametricReference`, `PrepareT3MaterialMeasure` and `NodalWallWeights`;
there is no alternate area, structural mass or contact-force implementation.

The adapter retains exact coordinates, source parent identities, native cyclic
connectivity and a complete mapping from sorted contact weights to family
indices. Contact areas follow the qualified Q4 center-area / T3 native-area
policy. They do not imply LS-DYNA contact equivalence, friction, thickness
offsets or general deformable/deformable contact.

Startup has explicit parent/node/byte bounds and publishes one immutable handle
only after every reference and weight succeeds. The peak payload budget charges
the complete native binding, active coordinate/map storage, temporary reference
preparation and the whole allowed weight budget. It is a conservative owned
payload limit, not measured RSS; allocator bookkeeping is excluded. Temporary
area objects disappear after the owning weights copy their values. No source
buffers are borrowed afterward.

The source-neutral owning target is `robo_dyna_shell_collection_contact_geometry`.
Its actual six-part gate runs in `robo_dyna_source_assembly_bindings_check`.
All eight assembly/startup/contact tests pass (2026-09-10), including two new
tests that independently enclose every parent and unique-node area using
long-double coordinate arithmetic and verify complete source mapping/lifetime.
Evidence: `crash-work/reports/source-assembly-contact-{configure,build}-1`,
`source-assembly-contact-build-2`, and `source-assembly-contact-tests-1`.
The first build found a test-only mixed-type `auto` declaration; it was fixed
without changing production arithmetic.

The actual 915-parent/1,030-node geometry has total reference contact area
0.16120413124345445 m2 and maximum X -0.17174556999999999 m. Its conservative
startup payload budget is 6,212,240 bytes. Wall placement must use these complete
assembly bounds. This host geometry gate does not initialize a CUDA contact
owner or establish finite-wall coverage for a future trajectory.

`CheckWallCoverage` requires a motion box containing the complete collection
and projects both X endpoints onto the actual mesh plane before reusing TL
finite-edge/hole coverage. The actual original wall (62 vertices,100 triangles)
passes coverage with a20mm leading gap and10mm projected margin for this
assembly. Its represented X is-0.15174557m; source Y/Z, connectivity and IDs
remain unchanged. Invalid outside/incomplete envelopes preserve the prior
certificate. `source-assembly-wall-{configure,build,tests}-1` passes both CTest
groups (nine functions); this certifies the declared envelope, not future
trajectory containment. ArtifactIO now has one shared owning CMake definition
used by the reader, wall adapters and production output.
