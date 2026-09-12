# Two-sided fixed-feature penalty algebra

This is a value packet, not an admitted contact contributor. Its two endpoints
are frozen linear weights over three or four original parent nodes. They may
be native T3/Q4 shape values or physical-facet compositions. The evaluator reads
all current x/v values in original slot order, including zero weights. No
virtual node, mass, clock, owner, feature search, area or local timestep exists.
Source identities and reference-plane admissibility remain the S0/facet
binding's responsibility.

With represented endpoint positions a,b, the declared gap must equal binary64
`(hypot(hypot(a.x-b.x,a.y-b.y),a.z-b.z)-hA)-hB`, with signed zero equivalent.
The normal is the represented componentwise division `(a-b)/d`. A zero distance
rejects; no fallback normal is fabricated. Nonnegative constant reference half
thicknesses enter this distance radius only; Q4 approximation errors do not.
The positive coefficient k is explicitly supplied, with no area/material rule.
The existing `normal_contact_detail::ApplyPenalty` supplies zero-damping normal
force and energy. Its finite arithmetic and unilateral touching convention are
retained; no unused mass/damping/timestep fields are exported.

Pair forces are exactly opposite represented vectors. Natural-slot nodal forces
use the existing `Scale(force,weight)` order. Shared nodes merge A slots then B
slots before sorting; duplicate masks must agree. Full reaction-bearing force
and projected free-component force are distinct output fields. Zero direct
couples are retained. Moment and virtual-work identities are algebraic, subject
to floating-point error and the existing 1e-12 partition admission; free forces
alone need not conserve momentum when reactions are removed.

The bound uses the **represented per-occurrence** vectors `Scale(+n,wA)` and
`Scale(-n,wB)`, merged by S1 before mask projection and bounding. It does not use
the different scalar-weight-merge schedule. Nodal force multiplication and
represented J multiplication are separately rounded; no bitwise discrete
derivative identity is asserted. Positive underflow in a represented J product
or an outward bound, and overflow, reject without output publication. Inactive
pairs return a zero normal stiffness; touching retains the one-sided active k.

For mathematically fixed linear weights, active energy is `U=.5*k*(d-hA-hB)^2`.
Its separation Hessian is `k*n*nT + k*gap/d*(I-n*nT)`. The second term is negative
in overlap and is **not represented by the frozen normal majorant**. A host
finite-difference control demonstrates this limitation. This slice provides no
absolute/nonlinear stability, changing-feature derivative, actual CIN/body
congruence, CCD, contact ownership or full self-contact completion claim.

All public output is success-only. Borrowed allocation extents, nonoverlap and
execution-memory-space lifetime are caller contracts. Arithmetic requires RN
binary64, no FMA contraction, reassociation, fast-math or flush-to-zero. The
packet owns only bounded values, with no dynamic allocation or shared buffers.

## Qualification

Six host functions pass: exact analytic force/energy/moment/work; 42 T3/Q4 nodal
energy-gradient differences with fixed weights; shared-node cancellation,
partial masks and the reused independent exact-dyadic bound/body-congruence
oracle; inactive/touching/signed-zero gap; eight late invalid/overflow cases,
whole-output preservation and retry; negative tangential curvature control.

The optional CUDA function is authored for 12 concurrent cases plus 12 retries,
comparing all named packet fields bitwise for supported controls and checking
the independent exact bound after readback. Includes an exact 3-4-5 distance,
shared masks, late NaN, gap mismatch, zero distance and bound overflow. It has
not been compiled or executed by the author. No native donor solver is invoked
for this algebraic utility.

Root commands (inside the normal shared qualification guard):

```sh
cmake -S <TL>/lib_utest/qualification/surface_penalty_pair -B <build> -DCMAKE_BUILD_TYPE=Release -DSURFACE_PENALTY_PAIR_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build <build> --parallel 4
ctest --test-dir <build> --output-on-failure
```

Owning Bazel targets: `//lib_utest/qualification/surface_penalty_pair:host_check`
and the dependency `//lib_utest/qualification/weighted_surface:host_check`.
The existing S1 qualifier remains unchanged numerically; its BUILD gains only
an additive exact-oracle library for reuse. Existing surface/Q4/T3 production
and all their historical receipts remain untouched.
