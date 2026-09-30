# Bounded large-rotation and localized-spin qualification

The native recurrence now passes QEPH and T3 yielded load/hold/unload paths with
**1.64794921875 rad** of superposed world rotation, at the existing h and h/2.
The analytic endpoint position, midpoint velocity (including Omega cross x), and
nodal spin still come from one prescribed path. Native and production histories
remain independent. No material, force, time-step or application guard changed.
This extends the selected native arithmetic comparison beyond one radian; it is
not an unrestricted objectivity proof, a physical validation or a coupled crash.

`YarisLocalizedSpinFixture.h` additionally retains the exact original unflattened
coordinates/connectivity of EIDs2214871/2214872 and material/section2000145,
curve2100180. Header comments give the original source-member and authenticated
inventory hashes and source line ranges. A common local frame and shared-node
anchor make the synthetic preload's x/v/director path compatible across both
parents. Each native/production three-point history yields before the spin test.
This preload is deliberately **not** the recorded impact history.

The fixture then holds all positions, with zero translational velocity, and
prescribes continuous director rotation of the shared NID2181592 at 7800 rad/s.
The initial director can be continued as exp([omega]x * elapsed) times the
preloaded director; midpoint omega is constant. This is a kinematically valid
prescribed rotation, with no claim that held positions satisfy free-node dynamics.
A 0.244140625 ms spin segment reaches **1.904296875 rad**, with h=2^-23 s and h/2.
Every interval compares full native geometry, forces/couples, HOURG, signed work,
thickness, point stresses, PLA and rate histories to the production adapter.

| Post-preload mechanism metric | h | h/2 |
|---|---:|---:|
| Parent normal separation, rad | 0.1679769518 | 0.1679769518 |
| Own-normal vs zero-spin maximum point stress difference, Pa | 0.004937589 | 0.004816115 |
| Common-axis vs zero-spin maximum point stress difference, Pa | 2.828954632e9 | 2.828954632e9 |
| Maximum absolute common-axis carried internal power, W | 42560.60251 | 42560.59256 |
| Maximum absolute signed HOURG-work difference, J | 0.002770468711 | 0.002770468664 |
| Maximum absolute accumulated strain | 0.1222411634 | 0.1222421064 |
| Maximum thickness times curvature | 0.01414906792 | 0.01414907138 |
| Minimum thickness/reference thickness | 0.9940827218 | 0.9940827184 |
| Maximum area/reference area | 1.0063842886 | 1.0063842886 |
| Maximum h/native unscaled DTEL | 0.1168586220 | 0.0584293110 |
| Maximum displacement from original nodes, m | 0.00137109362 | 0.00137109362 |

Each parent's own normal is its IDRIL0 near-null direction. A single common
shared-node axis has nonzero tangential components for both warped parents.
The second case develops substantial transverse stress and internal power even
though the axis is mostly normal. Signed HOURG work is neither labelled purely
dissipated energy nor added again to material work. These numbers belong to the
synthetic constrained-motion experiment, not the actual impact: the recorded
8h terminal normal separation was about 0.0667 rad, not this preload's 0.168 rad.

For the application domain decision, this evidence supports treating total
carried quaternion angle separately from native deformation and projection
measures. It does **not** support dropping normal spin, its torque coupling or
native rotary inertia, or inferring safety from a high normal-spin fraction.
The actual paired-source trace/native comparison remains necessary, followed by
an explicit reviewed application domain change and the next controlled impact
prefix. Existing area/thickness/strain/curvature/native-DTEL checks stay intact.
No application acceptance decision is made by this qualification.

## C3 element rate versus material-point filtering

The separate filters are intentional native branches. Retained
`../../native/t3/original/engine/source/elements/sh3n/coque3n/c3forc3.F`
lines546–556 sets `asrate=one`, computes the current element energy-equivalent
rate before thickness update, then overwrites `gbuf%epsd`. Its SHA256 is
`311455e0ce9c40399f7651b5dbc3937725edd1494753c15b9975e0fcdf842b7f`.
`NativeLayeredT3Section.F` preserves this element diagnostic overwrite.

Separately, `original/mulawc.F90` lines695–701 selects the point filter coefficient
from PM(9)*dt, bounded by one. LAW44 VP2 in
`../../native/law44/original/sigeps44c.F` lines204–221 mixes raw EPSD_PG with each
point's own old UVAR1, then publishes that point's filtered history. The complete
SIGEPS44C SHA256 is
`2e620f9b00f73b162fdc8a57e20f749ad01639d481770b21039145f28618b120`.
Existing native cutoff packing resolves PM(9)=2*pi*cutoff_Hz. Thus ASRATE1 at the
C3 element level does not disable the constitutive filter or add a second filter
to EPSD_PG. The extended T3 test checks the native element diagnostic equals the
raw section rate and that the first active filtered point value is different.

## Gate

The existing `shell_layered_native_recurrence_check` target now has five test
functions. The new source-pair function, large-rotation scenarios and existing
phase/reset/late-failure controls pass under one CPU and a 512 MiB address-space
limit (about 1.25 seconds for the five functions, no GPU). Numeric GTest
properties are written as full-precision strings so diagnostics cannot silently
truncate through GTest's integer overload. The initial metadata-only integer
conversion output was superseded without changing mechanics or tolerances.
