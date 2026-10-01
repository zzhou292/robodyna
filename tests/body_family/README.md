# Body-family rename baseline

`//tests/body_family:baseline_test` runs ten bounded CPU cases against the existing
auxiliary-frame body and eight Easy-body types. The same legacy names exercise
reverse aliases after the implementation moves. Tests cover constructor overloads,
analytic mass/inertia, hull/mesh topology, factory tags, adjusted base pointers,
shared ownership, copy/clone behavior and supported object archive fields.

The baseline passed before the family rename at parent `42f26e9fef`:
`crash-work/reports/robodyna-body-family-baseline-2.json`. The existing frozen
BodyEasyBox and auxiliary-body byte fixtures remain separate and unchanged.

Archive checks require exact stored reference-to-COM position and quaternion.
The opposite transform is computed from a cached rotation matrix. The unchanged
constructor initializes that matrix from eigenvectors, while archive input rebuilds
it from the stored quaternion. The pre-rename mesh example differs by four ULP in
x and one ULP in z in all three archive formats: approximately
`[-1.7763568394002505e-15, 0, 8.8817841970012523e-16]` metres. Only that derived
getter uses component `EXPECT_DOUBLE_EQ` (four ULP); stored fields stay exact.
The initial failure and diagnostic receipts are retained as
`robodyna-body-family-baseline-1.json` and
`robodyna-body-family-frame-diagnostic-1.json` in the outer reports directory.

The two hull constructor archives have an existing `m_mesh` writer / `mesh`
reader field mismatch. Constructor, topology and factory tests cover those types;
this suite does not claim their text-archive roundtrip works or that any of these
object archives supplies physical restart.
