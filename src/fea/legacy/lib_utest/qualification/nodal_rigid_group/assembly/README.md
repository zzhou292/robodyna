# Ordered rigid-part startup values

This qualification owns a value-only startup seam for original MAT_RIGID PART
bodies, optional extra nodes, and a raw Iflag=2 child merge. It does not attach a
body to an owner, supply source membership, generate missing coefficients, or
advance constrained nodes. All inputs are explicitly supplied SI values; native
member scalar inertia can be zero in this value layer. The existing owner and
plain-group admission contracts remain unchanged.

`PrepareAssemblyRawBody` preserves separate ordered PART and extra-node lists.
It reproduces the selected 3D ICOG=1 branch, including INIRBY line 308 setting
the primary shift origin to the new center after extra-node accumulation.
`MergeAssemblyRawBodies` consumes raw tensors and retains both original primary
mass/inertia contributions. `FinalizeAssemblyRawBody` runs the existing Eigen
factorization and native Ispher=2 correction once at the final root. Eigen axes
are an equivalent tensor factorization, not native VALPR bitwise axes.

The inputs require at least two PART nodes, positive explicit primary mass/J,
nonnegative member M/J, finite source-unit conversions, and bounded counts.
No skew, supplied inertia, SPC, sensor, imposed primary position, 2D, or general
hierarchical execution is admitted by this seam. Output is unchanged for
invalid ranges, overlapping output/input storage, overflow, and failed
factorization. There is no allocation in the three value operations.

## Native authority and distinct ordering contracts

All donors are complete files from OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`. `native/source-manifest.json` records
their byte counts, SHA-256 and git-blob identities. `verify_sources.py` checks
those files and every exact included fragment before the owning native build.

- INIRBY: PART center 189–213; extra center 292–309; primary tensor 327–341;
  PART tensor 349–366; extra tensor 452–469; merged center 510–516; parent
  transport 526–540; child raw tensor 570–584. The source mass guard at 202
  is enforced by the C++ admission before division; the packet wrapper receives
  already admitted inputs. Root-only principal correction is 615–845.
- `convertutils.cxx` 368–403 accumulates the generated PART centroid in
  first-seen unique-node element traversal order. Its 475–522 node extraction
  sorts/uniques the node union; `hm_lecgrn.F` 539–570 emits PART members in
  increasing internal-node index. These are distinct orders. This value seam
  takes the supplied original primary position and native ordered members;
  it does not establish the complete converter/parser traversal.
- `convertelementmasses.cxx` maps ordinary ELEMENT_MASS to ADMAS type 5.
  `hm_read_admas.F` 429–470 adds that value to MS and TOTADDMAS at 466–467,
  with no IN update. The native packet test checks this producer contribution;
  it does not establish final auxiliary-node IN after other producers.

## Decisive checks and limits

Four host functions compare ordered full tensors against an independent
long-double scalar quadratic oracle, make the extra-center branch and two
original primaries observable, demonstrate why Ispher correction must follow
the merge, and test rejection/unchanged output/exact retry. Three native
functions compare original PART/extra and pair-merge packets with the pinned
Fortran arithmetic, exercise both ordinary and measurable primary values,
check the existing Ispher fragment, and qualify the point-mass increment.
Strict floating-point flags are required; no production reference produces
native histories or native raw input tensors.

Author checks: four host functions PASS under one CPU/512 MiB; all new C++
units pass syntax; five donors and ten fragments authenticate. Native runtime
and existing owning regression qualification are intentionally root-scheduled.

From the TL root, using the existing qualified GNU Fortran compiler:

```sh
cmake -S lib_utest/qualification/nodal_rigid_group -B BUILD \
  -DCMAKE_Fortran_COMPILER=GFORTRAN -DTL_NODAL_RIGID_CUDA_CHECKS=OFF \
  -DTL_NODAL_RIGID_OWNER_CHECKS=OFF
cmake --build BUILD --parallel 1 --target nodal_rigid_assembly_check \
  nodal_rigid_assembly_native_check nodal_rigid_group_check nodal_rigid_group_native_check
ctest --test-dir BUILD -R 'nodal_rigid_(assembly|group)(_native)?_check' --output-on-failure -j1
bazel test //lib_utest/qualification/nodal_rigid_group/assembly:nodal_rigid_assembly_check
```

The source Yaris census is 22 original primitive bodies/regularizers and two
disjoint merges producing 20 roots. Its 54 auxiliary mass nodes, other source
coefficients, exact source closure, and eventual owner admission remain separate
requirements. Passing these packets cannot establish a complete vehicle model.


Root integration is TL `9c0755a`. CMake/native gates pass seven new functions
and31 existing rigid startup/step functions. Owning Bazel builds and its four
value functions pass. See `crash-work/reports/rigid-part-assembly-tests-1` and
`rigid-part-assembly-bazel-{build,tests}-1`. The shared finalizer also participates
in the passing actual six/seven-part CUDA binary capture gate
`component-binary-frame-tests-1`. Raw body values do not admit new source bodies.
