# Represented Jacobian diagonal majorant, S1

Pure allocation-free host/device values. This increment adds no contact force,
surface search, owner, mass, stiffness assembly, timestep or runtime admission.
Geometry, thickness/area/support ownership, active-set changes, tangent curvature
and full nonlinear stability remain outside this profile.

## Exact object being bounded

`BuildRepresentedJacobianMajorant` consumes up to eight represented per-occurrence
`Vec3` terms, source node indices and fixed XYZ bit masks. For each node it merges
component values by ordinary binary64 additions **in supplied occurrence order**.
Duplicate nodes must declare identical masks. Only after merging does it sort
nodes, set fixed components to positive zero, and bound the resulting represented
vectors. Cancellation is therefore the actual rounded merge, not an exact-real
sum silently substituted for that merge. Unfixed signed zero bits survive.

`BuildSignedNormalMajorant` separately implements the existing scalar normal
schedule: merge signed weights in supplied order, then multiply the common
represented normal once per node. It reuses `SignedNodeWeight` and the existing
normal tolerance (`abs(hypot(n)-1)<=1e-12`), and never renormalizes the normal.
Scalar-before-vector and vector-before-merge are distinct rounded constructions;
a future consumer must select the operation matching its actual Jacobian/force
producer. Neither is a substitute for authenticating that producer.

The existing six-node `NormalJacobian`, `BuildNormalJacobian`, mass view and their
source files are unchanged. The new eight-node packet retains canceled and fully
fixed rows, which carry zero majorants. A valid zero matrix is allowed, including
zero stiffness; it does not establish any dynamic DOF or contact response.

Let `P_i` be the diagonal free-component mask and `J_i=P_i J_i` the **represented
merged vector**. The existing scaled `mass_detail::UpperNorm` produces
`a_i >= ||J_i||`. Existing upward additions give `A >= sum_i a_i`. Existing upward
products give `D_i = up(up(k*a_i)*A)` for finite `k>=0`. All these bounds treat
their binary64 values as exact real numbers after the represented merge.

For arbitrary real vectors `v_i`, weighted Cauchy and the per-node norm bound give

```
k (sum_i J_i dot v_i)^2
 <= k (sum_i a_i ||P_i v_i||)^2
 <= k (sum_i a_i) sum_i a_i ||P_i v_i||^2
 <= sum_i D_i ||P_i v_i||^2.
```

Zero `a_i` contributes no division or exceptional case. Thus the block matrix
`diag(D_i P_i)` dominates the frozen represented rank-one matrix `k J^T J`.
It also dominates after **any exact congruence** `B^T (...) B`, including a
represented body/free-node linear velocity map. That algebra does not verify
the actual owner/CIN/group map, its floating reductions, coefficient phase,
constraints, or subsequent timestep policy. Those require a separate gate.

Using `|weight_i|` in place of `||J_i||` is invalid for merely tolerance-admitted
normals. The exact negative control uses `n_x=nextafter(1,+infinity)`, signed
weights `+/-1/2`, `k=1` and velocities `+/-1`: the old `D_i=1/2` underestimates
the represented quadratic form. The new bound passes without a normal tolerance
change or a tunable error multiplier.

## Arithmetic and failure contract

Inputs/output are borrowed, nonoverlapping, sized ranges in one execution memory
space. No value implies allocation, source, owner or publication authority.
Required arithmetic is IEEE binary64 round-to-nearest basic operations and sqrt,
no contraction/reassociation/fast-math/FTZ. Source-order summation is retained even
when an exact-real sum would be more accurate. Counts above eight reject before
term reads. Invalid masks, inconsistent duplicate masks, nonfinite data, invalid
node indices or stiffness fail. Fixed/zero terms are still validated.

The existing outward helpers preserve exact zero and explicitly reject a positive
product or norm component lost to underflow. Some subnormal matrices are admitted
(e.g. squared `1e-160`); some finite extreme vectors are rejected because a scaled
norm term underflows, even if the dominant norm is finite. An outward overflow,
including the final product or sum, also rejects. The common-normal adapter
rejects a nonzero scalar/vector product lost to zero before bounding. All
failures leave the complete output unchanged; no truncated/zero substitute bound
is published. The packet has no clock and no independent transactional state.

## Independent qualification

`ExactOracle.cpp` converts each finite binary64 value explicitly to an exact
dyadic rational using its binary significand/exponent and Boost `cpp_rational`.
It uses no production interval helper, float sqrt, decimal conversion, or copied
rounding algorithm. It verifies `a_i^2 >= J_i dot J_i`, the sum/product bounds,
and exact rational quadratic forms of the **represented merged** output. Input
merge/node/mask identity is checked separately against a map-based reference and
explicit source-order cancellation fixtures. A deliberately corrupted old
weight-only diagonal is rejected by the same quadratic-form oracle.

Six host functions include 128 signed/masked supports with 768 exact quadratic
probes; 49 exact body/free-node congruence probes; eight-node and shared-node
support, cancellation, partial/all-fixed masks, positive/negative zero, normals
slightly longer than one, representable subnormal bounds, underflow/overflow,
late invalid inputs, output preservation/retry, and legacy six-node comparison.
The optional CUDA test compares all represented output bits and statuses for 16
direct/common-normal, signed, masked, subnormal and failing packets. Host and
device results also go through the independent exact oracle. CUDA is authored
but not executed by the author.

## Commands

Host author gate from workspace root, 1 CPU/512 MiB:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/surface-majorant-author.json \
  --lock crash-work/reports/self-contact-majorant-author.lock \
  --cpus 1 --min-available-gib 1 --max-rss-gib .5 --timeout 180 -- \
  bash -c 'cmake -S crash-work/worktrees/self-contact-majorant/lib_utest/qualification/surface_jacobian_majorant -B crash-work/build/surface-majorant-author -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS_RELEASE="-O0 -DNDEBUG -g0" && cmake --build crash-work/build/surface-majorant-author --parallel 1 && ctest --test-dir crash-work/build/surface-majorant-author --output-on-failure'
```

Root-only owning CMake gate (through the shared bounded execution guard):

```sh
cmake -S Total-Lagrangian-FEA/lib_utest/qualification/surface_jacobian_majorant \
  -B crash-work/build/surface-majorant-root-1 -DCMAKE_BUILD_TYPE=Release \
  -DSURFACE_MAJORANT_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build crash-work/build/surface-majorant-root-1 --parallel 4
ctest --test-dir crash-work/build/surface-majorant-root-1 --output-on-failure
```

Owning host Bazel target:
`//lib_utest/qualification/surface_jacobian_majorant:host_check`.
Installed Boost headers are a qualification-only dependency. Production is
header-only, target `tl_surface_jacobian_majorant`, and changes no existing
numerical helper or historical source receipt. Passing this gate does not admit
S1 into any physical participant or establish deformable self-contact stability.
