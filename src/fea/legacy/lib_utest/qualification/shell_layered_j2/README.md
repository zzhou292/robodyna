# Three-point tabulated shell section

This opt-in TL value path composes the shared tabulated plane-stress point law
with the existing QEPH and T3 prescribed force operations. It adds no owner,
clock, contact law, allocation, or commit. Existing LAW1 files and native
`History`/`ForceTrial` layouts are unchanged, including T3's 976-byte trial.

## Composition and publication

- `elements/sections/ShellLayeredJ2.h` owns three point histories, integration,
  and section diagnostics, including optional per-point source rate history. Inputs use material order XX, YY, engineering XY,
  YZ, ZX, KXX, KYY, KXY. `ShellLayeredJ2Work.h` applies the existing generalized
  stress work and instantaneous membrane viscosity once.
- `qeph/QephLayeredJ2.h` and `t3/T3LayeredJ2.h` reuse each family's geometry,
  rate preparation, material GS, stiffness estimate, force projection and
  history validation. Their value wrappers pair the ordinary history with a
  separate section sidecar. Exact integrated material FOR/MOM must agree at
  entry. The caller binds the immutable prepared point material and curve.
- `InitializeLayeredJ2History` admits zero material history only.
  `EvaluateLayeredJ2Force` stages the entire ordinary trial, section and
  diagnostics before returning success. The caller must publish
  `force.proposed_history` and `proposed_section` together, under the existing
  owner/batch publication boundary. Failure never publishes either component.
  Replacing curve pointers during host/device upload is a caller operation;
  pointed-to values must remain identical and immutable throughout the run.

## Exact selected donor section

The retained OpenRadioss ordinary centered NIP=3 table has positions
`(-.5,0,.5)` and force weights `(.25,.5,.25)`. Its separate moment weights are
`(-.0833333,0,.0833333)`, with default-real literal rounding followed by MYREAL8
promotion. C++ uses `static_cast<double>(.0833333f)` and the native table probe
checks this exact value. These weights are neither Gauss quadrature nor WF*Z.
Do not substitute an exact 1/12: the donor elastic bending limit differs from
that value by a small, intentional relative error below 6e-7.

For each point, the in-plane strain increment is membrane plus
`position * reference_thickness * curvature`; transverse shears share actual
GS. Integrated FOR is `sum(WF*stress)`, MOM is `sum(WM*stress)`, both stored in
Pa. Existing projections multiply them by the interval's effective force
thickness and its square. The source ITHICK=1 adapters use accepted reported
thickness for that effective thickness, coefficient preparation and layer
positions. Reported thickness starts at its previously accepted value, then
receives each point's elastic and plastic thickness additions separately, each
multiplied by `WF*effective_thickness`. The original reference identity and
native structural mass/J remain unchanged. The section's legacy input name
`reference_thickness` means this interval's effective force thickness; it does
not require the original startup thickness.

`plastic_work_density_increment` is the WF-weighted native point diagnostic
in J/m3, not total work or an additional energy term. Multiplying it by actual
current shell area and accepted effective thickness gives its per-element joule
increment. Existing generalized old/new stress work remains the total work
ledger. Maximum/mean plastic strain, mean/minimum point tangent ratio and
mean/last-point pre-update yield are distinct diagnostics. The elastic native
stiffness estimate remains the stability bound; no softened timestep is used.

**QEPH stabilization uses the native NPT3 plastic correction.** After the common
elastic increment, `QephPlasticStabilization.h` applies CZFINTN1's mean/minimum
ETSE correction, global section yield criterion and unloading logic. It uses
the last material point's SIGY, matching the source caller output; the weighted
mean yield is a separate diagnostic. Corrected stabilization is projected into
forces and work once. Elastic stabilization work remains in generalized
internal work, while the native TESY/EVIS(8) viscous contribution remains in
the separate `hourglass_viscous_work` channel. T3 has no QEPH stabilization.

The optional rate branch uses the shared LAW44-derived point law. The source
Yaris configuration explicitly enables Cowper–Symonds C=8000/s, P=8 and the
pinned direct-import cutoff of 10000 Hz. The section computes the native total
equivalent rate from membrane/bending increments and accepted reported
thickness. Each point receives that scalar and carries its own filtered-rate
history. Neither a filter nor a material history advances at initialization.
See [RateReference.md](../native/law44/RateReference.md) for the exact default
resolution and source evidence. Explicit rate-off experiments remain available.

Failure/deletion, curve extrapolation, kinematic hardening, arbitrary MAT024
options, self-contact/severe-folding qualification and restart remain outside
this branch. Full native layered-element recurrence and time-dependent
prestressed-rotation qualification are follow-up gates; this section is not a
claim of complete LS-DYNA or OpenRadioss equivalence.

## Donor mapping and scope

All donor sources use the same OpenRadioss source pin as the existing native
qualification. Exact byte identities for the bounded source inspection:

| File | SHA256 | Used operation |
|---|---|---|
| `coqini.F` | `610d39a9c2280e5fdd2a0a0c58586e17d20fcb96c4c54bda47d8eec8fcd0d769` | COQINI Z0/WF and separate COQINI_WM, NIP3 |
| `layini.F` | `744be8fb1910479bd5d29c3e0f6e421195a09b24c3e19991969a1516000da7c3` | TYPE1/9 section weights and positions, lines 244–257 |
| `mulawc.F90` | `778df028efcdb9e28e8b2cb9156a431937c06fa073260d2c002164b420848526` | Interval THK0 layer strains; THKLY/WMC integration, lines 765–811 and 2657–2664; generalized work, lines 3046–3093 |

Point arithmetic and two thickness contributions are owned by
`materials/TabulatedShellPlasticity.md` and `qualification/native/law44`.
The latter retains and hash-checks the actual COQINI source, compiler settings,
and native point routines. Existing QEPH/T3 source maps remain authoritative
for reused geometry, viscosity, stabilization, projections and work.

## Focused gates

The host target compares the actual native table and independent native point
updates over a persistent four-step mixed membrane/bending/unload sequence,
checks the elastic bending limit, third-layer rejection with unchanged output
and exact retry, excludes plastic-work double counting, and exercises both
family force adapters with mismatched-sidecar and epoch rejection. The CUDA
target evaluates both adapters on one thread, carries their accepted histories
through two plastic intervals, compares host/device results, and verifies a
late third-point failure and clean retry. Its sole device packet is below
16 KiB; it starts no dynamics or full source-part simulation.

The separate native rate suite carries three native point histories and its
own thickness through 1664 loading/hold/reverse intervals. A native CZFINTN1
wrapper checks plastic/global-yield stabilization, elastic unloading and the
high-yield sentinel. Resident mixed-family tests cover common publication,
late rejection/exact retry and immutable material/rate scope.

The added covariance tests preload actual source-curve yielding, rebind those
local histories under a constant world rotation/translation, then independently
carry both paths through loading, hold and reversal. They reuse the existing
2e-11 covariance coefficient and dimensional force/history scales. Planar,
warped and skewed QEPH plus three T3 shapes check rotated forces/couples,
unchanged local stress/PLA/filter histories, evolving thickness, work and exact
section/resultant association. These are constant-frame covariance tests;
they do not substitute for a time-dependent rigid-spin or complete native
layered-element recurrence test.

The integrated pre-covariance build passed all 36 CTest groups in
[plastic-delivery-tests-1.log](../../../../crash-work/reports/plastic-delivery-tests-1.log).
Both host covariance functions also pass in the bounded native recurrence
build described below. This directory can be configured directly; `TL_SHELL_LAYERED_J2_ENABLE_CUDA=OFF` selects only
host checks. The workstation's serialized build/run
owner performs execution and records evidence.

The optional [independent native recurrence](native_recurrence/README.md) now
composes complete native QEPH/T3 leaves with NIP3 physical LAW44 thickness,
stabilization and signed work. Its prescribed yielded rotation/unload gate
carries separate native histories; the earlier constant-transform covariance
checks remain a distinct qualification.
