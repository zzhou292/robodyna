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
Their original cohort constants stay distinct: Starter MVSIZ512 and selected
GNU/Linux Engine MVSIZ129. A 143-primary fixture crosses the latter boundary.
Foreign-partition exchange is explicitly outside the local reference domain.

The reference is serial and bounded to 256 nodes and 160 primary faces; these are
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

CMake owns eleven host GTest groups, the production-only consumer and two source
checks. Bazel additionally owns `consumer` and `source`; it does not substitute
for the complete native oracle gate. Source authoring and source generation do
not establish compiled or numerical qualification. Integration into a physical
source factory still owns primary ordering, fixed activity, material/gap fields,
removal topology, and physical accepted/trial lifetimes.

Observed phase boundary: observation2 captures positions at cycle0 but its
classification/boundary numerical fields at cycle216 (TT140.433us). Only the
immutable topology from that later capture is compared as startup evidence.
Every produced Starter/all-active-ready normal and bisector bit still compares
to the independent native oracle on the exact observed input coordinates. The
three later minus-zero X channels on main4 are preserved as phase evidence;
there is no sign normalization or tolerance substitution. Dynamic ACTNOR/TAGNOD
normal maintenance remains a separate runtime dependency before general fixed
nonplanar or moving-main equivalence claims.


Explicit general ordinary-shell policy
--------------------------------------
GeneralTopologyTest adds native valence3/4, winding, full-main candidate-prefix,
extra-vertex, disconnected-reference, strict score-tie, SI/floor, origin/cap and
failure-preservation coverage. SelectorOracle calls the original double selector;
its generated read-only hook returns actual ANGLE/YJNI values. Native IRR11
warning1245 is counted with its source edge instead of being mistaken for a
fatal qualification-stub error. Other native diagnostics remain fatal to this
bounded oracle. No donor arithmetic is replaced.

The current_normals owning project additionally builds
`type25_general_topology_normals_cuda`: real general-policy output and original
native startup output feed independent repeated NORMP states. Existing ordered
stage kernels and activation fixture are reused. These are source-authored gates,
not a claim that they have executed. A source-sized count-only Preflight test
records actual compiled forecasts without allocating or admitting a full Yaris.
General fixed-ready has zero/unavailable ready forecasts and remains rejected.


Resolved coating qualification
-----------------------------
CoatedStartupTest uses explicit supplied ordinary/forward/reversed Q4/T3 roles
and compares complete original SH2SURF25, I25NEIGH and I25NORM output. The wrapper
has one trailing C-int source-role array (23 C arguments); only marshalling and
scope metadata changed, never donor numerical bodies. Cases cover mixed roles,
source-order reversal, valence3/4, disconnected references, warp/rotation/SI,
signed zero and role provenance retained after identical normalized geometry.
Counted IRR11 warnings remain source-bound and do not delete rows. The new role
output/staging bytes are checked through compiled sizeof/Preflight. Default
ordinary scope, fixed-ready rejection, cap/alias failures and retry remain gated.
No case claims to derive actual coating signs from shell/solid membership.
