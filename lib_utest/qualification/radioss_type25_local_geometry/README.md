# Selected TYPE25 current local geometry

Source-only staging; qualification results belong to guarded owning receipts.
Donor a62b27e6. The public header is collision/RadiossType25LocalGeometry.h.

This covers selected local COR3 center/normal preparation and the complete
DST3_3 current raw geometry: Q4 center subtriangles, T3, normal interpolation,
free edges, vertex bisectors, ISHARP1 rounded/straight boundaries and native
weights. It does not generate/choose candidates, slide between main faces,
build native neighboring normals, supply material stiffness, or publish a
physical owner. Supplied lb/lc and source fields must come from those producers.

Supported profile: raw IGAP1, ISHARP1, INACTI5, IVIS2=1, local rows, no adhesion
or thermal work. Other modes reject explicitly. NOD_NORMAL stored slot ordering
is retained: T3 uses slot4 for its center; slots3 and4 need not agree.
Float32+float32 bisector addition rounds before promotion, as in native Fortran.
Normals are not normalized beyond the exact source operations.

Persistent contact geometry/history uses native working units. The production
bridge EvaluateNativeGeometryFromSi accepts explicitly scaled owner coordinates
and returns native geometry without a rounded-SI-history round-trip. The separate
EvaluateSiRawGeometry is an interoperability/output view. Final penetration/work/
forces convert at the owner boundary after native history arithmetic. There is
no direct-SI history formula masquerading as native operation-order equivalence.
NativeContactRow is the existing qualified P1.2 type, unchanged.

GeometryHistoryBatch is exactly one original logical native cohort, not arbitrary
regrouping of the entire vehicle. Source-owned secondary ID, row index, generation,
selected main and marker must agree with the selected trial row. The finalizer
does no second BeginHistory rollover. It stages the complete initial-offset max
pass, ordered offset pass and separate MAINF zero-penetration stiffness pass.
Repeated history targets retain native order; original cohort boundaries remain.

The bounded serial batch is a reference/qualification adapter. It is NOT the
production million-row CUDA design. The small private HistoryRow.h arithmetic
leaves are shared: a retained GPU owner assigns one writer per secondary row and
uses complete original-ordinal incidence, reusing OrderedNodeIncidence. Different
rows can run in parallel; the passes within each row retain native order. The
CUDA coupon exercises two independent row writers against the whole native cohort.

All input/source/range/capacity checks precede scratch/output writes. Scratch is
explicit, bounded and disjoint; outputs publish only after a complete successful
batch. Malformed input, late overflow or alias failure preserves both output
arrays. Input buffers must be valid, retained memory; an arithmetic packet does
not authenticate arbitrary pointers or create physical source authority.

The independent oracle compiles pinned original Fortran expressions, not the C++
translation. See native/README.md. All exposed native-defined fields, keys and
history slots are compared; unused source scratch is not invented. The wrapper
observes XP assignment readiness and rejects an undefined native read.

Owning gates:
- 139 finite branch packets, analytic Q4/T3 weights, native float32 sum witness,
  both normal sides, boundary interpolation/rounded choices and malformed inputs.
- Native two-pass history, repeated targets, negative-zero preservation, next
  cohort, exact cap, aliases, late identity/arithmetic failure and retry.
- Native/SI dimensions and near-cancellation against the native history oracle.
- Actual CUDA full corpus/permutation, raw and history failure preservation,
  independent row writers, and header-only host/CUDA consumers.
- Parent normal/friction owning tests run through the reused CMake subdirectories;
  no parent numerical code is changed here.
- Bazel owns header closure and pinned source preparation. It does not replace
  CMake's actual native Fortran/CUDA gates.

Example configure (root-controlled lane):
cmake -S lib_utest/qualification/radioss_type25_local_geometry -B <fresh-build>
  -DCMAKE_BUILD_TYPE=Release -DTYPE25_GEOMETRY_CUDA=ON
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local
Use the workstation guard for configure/build/tests, never a new vehicle run as
a local geometry debugging loop. No whole-contact correctness or performance win
is established by these coupons.
