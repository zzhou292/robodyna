# Ordinary-shell fixed-main startup qualification

The production path is a bounded host startup producer, so its owning gate is
C++/Fortran rather than an artificial CUDA implementation. The production-only
consumer links no native runtime, GTest, or CUDA. Precision flags are inherited
from the public CMake target. Numerical outputs and all float bits are compared
against the pinned independent native reference.

The oracle retains complete SH2SURF25, I25NORM, I25NORMP, I25FREE_BOUND and
transitive neighborhood helpers. The exact I25NEIGH prefix ends after every
retained topology, reference and LBOUND output; the omitted tail builds only
edge-contact/subsurface structures outside IEDGE0/NISUB0. PREPARE_SPLIT_I25's
original local reference-numbering and incidence loops establish the CSR.
Wrappers supply source-shaped local arrays and flags, never production math.
The source-defined Starter and Engine floor values are observed independently.
Foreign-partition exchange is explicitly outside the local reference domain.

The reference is serial and bounded to 256 nodes and 64 primary faces; these are
qualification limits, not the production capacity. Its original integer node
identities are bounded to positive INT_MAX. The public producer also admits
arbitrary unique nonzero 64-bit node/source identities. Tests cover mixed Q4/T3,
rotations, warp, scale, changed primary order, triangle unused slots, signed
zeros, both normal stages, SI conversion, capacities, rejected topology and
publication preservation. The observed 18-node/8-T3 wall fixture supplies input
mesh and expected native fields only; no captured values enter production.

Use the repository's bounded execution guard and the assigned heavy lane. A
fresh build can use the pinned local GNU Fortran compiler:

```sh
cmake -S lib_utest/qualification/radioss_type25_fixed_main_startup -B BUILD_DIR \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local
cmake --build BUILD_DIR -j4
ctest --test-dir BUILD_DIR --output-on-failure
```

CMake owns ten host GTest groups, the production-only consumer and two source
checks. Bazel additionally owns `consumer` and `source`; it does not substitute
for the complete native oracle gate. Source authoring and source generation do
not establish compiled or numerical qualification. Integration into a physical
source factory still owns primary ordering, fixed activity, material/gap fields,
removal topology, and physical accepted/trial lifetimes.
