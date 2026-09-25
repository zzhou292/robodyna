# Optional current-normal fields

`Input.current_normals` is a borrowed complete field view, not a physical owner
or a readiness receipt. Four face slots per main and one LBOUND/bisector record
per source normal reference must be present together. Partial, overflowing,
misaligned or mutually aliased field spans are rejected; every new writable
row scratch span must be disjoint from both field arrays before any write.

An all-zero descriptor preserves the existing static path and its predicates,
errors, source ordering and numerical arithmetic. A complete view overrides all
embedded main normals and legacy reference payloads. Those unused floats are not
read or checked, and the unused legacy reference pointer may be null. Topology,
IDs, coefficient/gap fields and reference counts still come from SourceView.
Only selected nonzero LBOUND records consume bisectors. No normal is normalized,
no signed-zero bit is changed, and unused T3 slots remain explicit caller data.

Startup and lifecycle reference types are aliases of one standard-layout shared
value type; the representation is unchanged and no unrelated-struct cast is
needed to bind current-normal producer output. The startup NormalView and public
fixed-main runtime ownership contracts are unchanged.

The moving owner keeps accepted/trial field arrays separate. BeforeNormals reads
no normal/bisector values. After the complete producer barrier, bind its staged
view once and keep it immutable through AfterNormals, CompleteRow and selected
geometry/response. Pair and opposite-side NewImpact factories use the same
accessors; Continuation and final geometry already pass through Pair. Input and
all other snapshot values must remain the same force-base state across this
boundary. A source generation is not a new mechanics clock or proof of barrier
completion. This slice does not broaden the existing fixed-main transaction.
