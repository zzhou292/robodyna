# Native reference placement in QEPH/T3 force values

This qualification binds `ShellReferencePlacement` to each immutable native
reference. The first noncentered force composition is analytic LAW44,
FilteredZeroC, centered-property NIP3 with typed reference-plane shift, and TAB1
AnyPoint failure. The original windshield thickness is 0.00228 m, with both
TopReferencePlane and BottomReferencePlane exercised. Original source NLOC
resolution remains an application obligation; these tests do not import or
advance the 8,502 source glass shells.

`ReferenceInput` owns the placement value. History identity compares it exactly;
the shared family force cores take it from that reference for coefficients,
point positions, resultant checks, and work. No public interval contains a
placement override. Q4 startup retains the native offset-square thickness
inertia contribution. T3 retains its different native TYPE1 inertia expression,
which contains no offset-square contribution. Total native inertia remains
authoritative and is never reconstructed from diagnostic partitions.

The complete in-process binding inventory has one explicit placement word after
each parent's density/thickness/E/nu words. Pair/collection encoding versions are
6/7; the pair has52 words and a collection has `4 + 29*Q + 23*T` words.
QEPH records include immutable projection_working_length_m; QBAT encoding8
adds the same authenticated (currently default-only) field. All inline
and owned sizes, preflight, and complete equality use that layout. On the qualified
host ABI, each native reference/history record gains 8 identity bytes; Q/T
ForceTrial sizes are 1424/984 bytes. Existing arenas use actual sizeof values. This is not a
persistent archive schema. Existing material/source/accepted archive schemas
remain unchanged. Noncentered resident batches and other full force formulations
are explicitly rejected pending their independent qualification.

## Independent native composition

The donor revision remains `a62b27e6baa555d222a580d6218867d0be4d70b5`.
The new manifest authenticates the existing complete family adapters, exact
placement donor preparation, and this small native packet adaptation. It does
not repin any donor or change a legacy native entrypoint.

The scoped native material wrappers call the complete CNCOEF3B/C3COEF3 routines,
setting GEO(199) using the authenticated native IPOS operation before that call.
Native M%ZOFFSET feeds the unchanged complete stiffness path. Exact LAYINI and
SHELL_OFFSET_WM_INI quadrature feeds the existing native point/failure/work packet
with actual family M%SSP. No production coefficient, geometry, point-law, or
resultant implementation is used to compute the native reference outputs.
The old and placed centered native packets are compared field-by-field and
bit-for-bit as a separate default compatibility gate.

The force-state tests compare complete current geometry, shell histories,
forces/couples, stiffness/dt/work diagnostics, all distinct TAB1 point histories,
current force stress, native point diagnostics, thickness, removal, and two later
inactive intervals. CUDA kernels own and advance their histories, compare with
independent host native packets, and exercise both wrong-reference placement and
late-point rejection before an exact retry. No owner clock, source admission,
resident collection admission, contact deletion, or application publication is
introduced.

## Owning gates

Author checks are host-only under one CPU and 512 MiB. Native and CUDA checks
must run through the workstation's serialized root qualification lane.

```sh
cmake -S lib_utest/qualification/shell_placement_force -B BUILD_DIR \
  -DCMAKE_BUILD_TYPE=Release -DTL_PLACEMENT_FORCE_NATIVE=ON \
  -DTL_PLACEMENT_FORCE_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel 2 --target \
  shell_placement_force_values_check shell_placement_force_native_check \
  shell_placement_force_cuda_check
ctest --test-dir BUILD_DIR --output-on-failure -j1 \
  -R '^shell_placement_force_(values|native|cuda)$'
```

Bazel owns the host gate as
`//lib_utest/qualification/shell_placement_force:shell_placement_force_values_check`.
Retain the centered TAB1, Johnson failure-force, layered LAW1, complete binding,
expanded host collection, and resident centered regressions after integration.

## Root integration qualification (2026-09-10)

Complete placed force/reference integration passes11 functions (6 host,3 native,
2 actual CUDA), including all three declared planes, failure/removal histories,
and centered native bit parity. Reports: `shell-placement-force-root-*`.
Post-integration regressions pass9 original reference/source functions,13 resident
constant-failure functions and8 original-source CUDA flight functions. Reports:
`placed-reference-{source,resident,source-flight}-regression-*`.

Q/T reference sizes are now528/504 bytes. The complete resolved-source forecast
is543,479,220 B, including182,907,000 B of reference capacity; the historical
forecast is465,994,140 B. All326,082 resolved original references still succeed,
with23,563 unresolved parents retained. These are startup/reference results,
not whole-shell dynamics or resident glass admission.
