# Controlled Solid24 native reference

First milestone: complete authored native HEPH sequence; numerical execution is
pending. No production force/history file or vehicle profile is changed. The
native-only target invokes existing independent geometry/material, original
SZHOUR_CTL/SHOUR_CTL, unchanged SFINT3 and world rotation, SDISTOR_INI/S8FOR_DISTOR,
and actual SCUMU3 nodal assembly. No numerical leaf is replaced by a stub.

The first explicit profile is active LAW42 alpha2/no-Prony, JCVT1/ICP1/IINT2/
ISMSTR10, DN=.1, nu in[0,.48999], all-active, no thermal/ALE/rotation/deletion. Higher-nu SDLEN8 is
rejected before numerical output, not silently replaced by ordinary SDLEN3.
ANIM_N=0/IAD_GPS=0 are explicitly set in the reused context. No STRHG18/strain
output history coverage is claimed. Geometry remains the existing positive-
Jacobian native coupon contract; unsupported native geometry may fail closed.

This first bridge is explicitlySI-only. All dimensional inputs share that
working-unit system; original tonne/mm conversion and its dimensional floors
remain a separate gate.
`native/Packets.h` documents the qualification contract; NativeSupport.h maps its
flat93-double wire representation. Carried state has22values: material9(including
EINT density), controlled force-like FHOUR12, and independent cumulative
EINT_DISTOR energy. Ccomponent*4+mode differs from nativeFHOUR(1,3,4) ordering;
explicit loops transfer every field. A nonsymmetric12-seed test exposes swaps.
Nodal coordinates are expanded to real nativeMVSIZ129 strides; forces and nodal
stiffness return in original source order through the authenticated permutation.

Material slots preserve actual reader and no-Prony update statements, including
the distinct PM32/PM100 values and finalPM107. Original parent sources and exact
slice locations are pinned. Hourglass energy uses the real native EINT update;
an observation-only hook returns its dt*modal-power numerator without feeding
that value back. Distortion energy stays cumulative and separately reported.
Material/HG/distortion STI are captured before SCUMU3 quarters its actual input.
The returned eight nodal coefficients come from that real assembly routine.

The native dependency closure is complete, including link-onlySFOR_N2S3 required
by the fullSFOR_4N2S4 module. Existing HEPH geometry/material/reference/context
are pinned and reused, not copied. Preparation preserves every original
numerical statement and leading dimension; identifier changes reverse exactly,
and the single observer insertion is separately reversible. Source tests reject
byte/blob tampering. Native test successes are required before any mechanics
claim. The CMake target is `solid24_icontrol_native_test` with
`TL_SOLID24_ICONTROL_NATIVE=ON` and the existing GNU Fortran wrapper.

Seven authored numerical cases cover material slots, all-stage initial/translation,
nonzero controlled history and energy carry, all12historyslots, distortion/work/
forcebalance/nodalassembly, late numerical failure/repair and rejected input
with unchanged output. They test
this independent oracle, not a C++ implementation. After the native gate and
source review, implement separate shared C++ controlled leaves with native/CUDA
comparisons; S6 and LAW90 retain their distinct dispatch/STI obligations in the
workspace SOLID_ICONTROL_MECHANICS_GAP_AND_PLAN.md.
