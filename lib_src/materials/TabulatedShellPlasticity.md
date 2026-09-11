# Tabulated shell plasticity with explicit source rate settings

This pure host/device material point adapts the plane-stress shell branch of
OpenRadioss LAW44. The default experiment is rate independent; the explicit
optional branch applies filtered total-rate Cowper–Symonds scaling to the same
plastic-strain/yield-stress table. Failure/deletion, kinematic hardening,
nonlocal corrections and complete LS-DYNA equivalence remain outside this
branch. The existing LAW1 path is unchanged.

All donors are pinned at `a62b27e6baa555d222a580d6218867d0be4d70b5`:

| Source | SHA256 | Reused behavior |
| --- | --- | --- |
| `engine/source/materials/mat/mat044/sigeps44c.F` | `2e620f9b00f73b162fdc8a57e20f749ad01639d481770b21039145f28618b120` | Elastic predictor, virgin hardening, three Newton evaluations, plane-stress projection and split thinning |
| `engine/source/tools/curve/vinter.F` | `4ecc3e2dc2119112826bd0cacfcc423e4551cca4a66e063b210d352c00a8fdcd` | Left-segment interpolation at exact knots and arithmetic order |
| `starter/source/materials/mat/mat044/hm_read_mat44.F` | `79915c2f70251e92578385dcaa1ec085b020914851f83c5126e39d00affd58c1` | Elastic coefficient arithmetic |
| `engine/source/materials/mat_share/mulawc.F90` | `778df028efcdb9e28e8b2cb9156a431937c06fa073260d2c002164b420848526` | Equivalent-stress/plastic-increment work diagnostic |

The full OpenRadioss licensing notice is retained in
[the existing donor license](../elements/qeph/LICENSE.md). The native sources
belong only in the isolated qualification reference; production includes no
Fortran entry point or second time integrator.

The common branch is `MFUNC=1`, `CA=CB=0`, `YSCALE=1`, `FISOKIN=0`,
`OFF=1`, `INLOC=0`, with failure/tension limits inactive. The default has `CC=0`;
all disabled rate declarations must have zero C/P/cutoff values. Enabling rate
selects runtime `VFLAG=2, ISRATE=1`; the default `Legacy` policy requires
explicit positive C, P and cutoff. The separately selected `FilteredZeroC`
policy requires enabled=true, C=0, resolved P=1 and a positive cutoff. It
advances the same filter history while skipping strengthening exactly. It does
not reinterpret disabled declarations.
It prepares `CC1=1/C`, `CP1=1/P`, and `PM9=2*pi*cutoff` in donor order. Every
point updates its own accepted UVAR1 by
`r = alpha*EPSD_PG + (1-alpha)*old_r`, `alpha=min(1,PM9*dt)`, then multiplies
curve yield and nonvirgin curve slope by `1+(CC1*r)^CP1`. Virgin hardening stays
exactly E. No rate history advances before a real accepted interval.

For the original positive-LCSS, absent-LCSR Yaris declaration, converter C8000,
P8, VP0 and ISMOOTH1 select this LAW44 branch; starter maps VP0 to runtime VP2.
The direct converted model enters starter without an export/re-read; an absent
Fcut is read as zero and the starter resolves it to 10000/s. The application
must supply `{true,8000,8,10000}` explicitly. These source-reader choices are
provenance, not implicit material defaults or claims about other MAT024 cards.

`PrepareTabulatedShellPlasticity` validates a caller-owned immutable curve with
2–1024 points, zero initial plastic strain, strictly increasing abscissae and
positive nondecreasing stresses. It prepares the pinned elastic coefficients.
`UpdateTabulatedShellPlasticity` consumes accepted stress and accumulated
plastic strain, filtered-rate history, engineering strain increments and the element's actual
transverse shear modulus. It stages a result and publishes it only on success.
Pointers must refer to storage accessible to the executing host or device.
Prepared coefficients and curve values must remain immutable during use.

The interpolation cache is replaced by a fresh binary search with the same
strict knot-side choice. Existing preparation overloads select
`ShellPlasticityCurveContinuation::StrictDomain` and reject extrapolation,
including a candidate that crosses the final supplied strain. The explicit
rate-plus-policy overload accepts `NativeLastSegment`, which reuses the same
VINTER last-segment slope and expression beyond the endpoint. It adds no point,
clamp or plateau. This policy admits PLA only below the separate native default
EPSGM cap (`static_cast<double>(1e20f)`); the cap branch remains unsupported.
The material catalog retains this policy in complete identity and prepared
parameters. LAW1 and analytic declarations reject non-strict table policies.
The native projection freezes the old-point curve slope within an increment; this code
preserves that behavior and the native special `HS=E` at zero accumulated
plastic strain. The accepted projection comes from the **third evaluated**
Newton iterate; the next guess computed during that evaluation is not applied.
No extra yield clamp or material iteration tolerance is introduced.

Point stress order is XX, YY, XY, YZ, ZX in Pa. Input shears are engineering
strains. The yield criterion uses XX/YY/XY; YZ/ZX retain the elastic predictor,
matching this donor branch. The section owns point locations, membrane/moment
weights, actual thickness update, integrated stress work and stabilization.
Elastic and plastic thickness-strain increments are reported separately so the
section can preserve the two native additions to thickness. `tangent_ratio` is
native ETSE, not a complete algorithmic tangent.

The section computes native CZFORC3/C3FORC3 `EPSD_PG` from membrane and bending
increments, accepted reported thickness, and `dt/max(dt*dt,1e-20)`; each point
receives that same scalar. Source ITHICK=1 also makes the accepted reported
thickness the next interval's force/coefficient/volume thickness. The adapters
use a local coefficient-input copy, retaining original reference identity and
native mass/J. Section weighted mean/minimum tangent and mean/last-point yield
diagnostics remain distinct; native QEPH stabilization consumes the last-point
yield, not a substituted weighted mean.
For the admitted normal source E/nu, the shared elastic coefficient helper and
LAW44 coefficient preparation agree exactly: the two G division expressions
are binary scaling by two, and the A12 multiplication only reverses operands.
The adapters retain that shared coefficient helper.

`plastic_work_density` is the native MULAWC diagnostic
`0.5*(old_equivalent_stress + new_equivalent_stress)*delta_plastic_strain` in
J/m3. It is not total stress work or a guaranteed exact split into recoverable
and dissipated energy. Multiplication by section thickness weight and area,
and consistency with discrete resultant work, remain section responsibilities.
Here `delta_plastic_strain` is the rounded subtraction of accumulated new and old
PLA, as in MULAWC:2011. It can differ from the constitutive `plastic_increment`,
which retains the actual native point iterate. The failure-caller qualification
corrected this diagnostic's earlier use of that iterate; force/history/clock
semantics and existing archived results are unchanged by the correction.

An explicit trailing `input.element_active=false` selects native parent OFF=0.
Predictor, rate and hardening evaluation still run; plastic gather is disabled
and local thickness additions are zero. The default remains active. Point damage,
saved-stress masking and final section-resultant masking belong to the separate
failure-aware NIP3 caller, not this constitutive leaf.

Focused local tests cover invalid input and unchanged output, a late
curve-domain failure with exact retry, and seven 64-increment cyclic sequences
plus one rejected sequence on both host and device. The separate native point
suite qualifies the material equations and original source curve. Added rate
tests cover independent filter rise/decay/saturation, shared section input with
distinct point histories, invalid-input and late-layer atomicity, host/device
cycles, joined material mismatch and mixed-family rejection/exact retry. The
integrated source-rate plasticity gates passed in the workspace's
[36-group delivery run](../../../crash-work/reports/plastic-delivery-tests-1.log).
This records the completed bounded branch; broader element/folding qualification
remains separate.

## Analytic companion

The existing table API and supported table behavior remain available. The
optional [LAW44 analytic hardening](Law44AnalyticHardening.md) uses the same point
recurrence with an explicit tagged source SIGY/ETAN declaration; it adds no
synthetic table or separate history owner.

The explicit continuation qualification uses the three complete original FAIL=1
curves ending at .3, .4 and .5, with original positive-C/P settings and a separate
rate-off control. Its [qualification README](../../lib_utest/qualification/shell_hardening_continuation/README.md)
separates source evidence, host checks and pending native/CUDA execution. This
policy does not opt existing source adapters or archives into failure-aware
resident mechanics.

The [zero-C qualification](../../lib_utest/qualification/shell_filtered_zero_c/README.md)
adds the pinned starter CC0/CP1/ISRATE1/VP2 branch used by the original glass
and membrane declarations. This is a material-value capability; source
admission, TAB1 failure history and section placement remain separate gates.
