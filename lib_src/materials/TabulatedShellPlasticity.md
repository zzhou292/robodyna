# Experimental rate-independent tabulated shell plasticity

This pure host/device material point adapts the plane-stress shell branch of
OpenRadioss LAW44. It is a rate-independent experiment using the original
plastic-strain/yield-stress table. It does **not** implement the source MAT024
C/P/VP rate response, failure/deletion, kinematic hardening, nonlocal corrections,
or complete LS-DYNA equivalence. The existing LAW1 path is unchanged.

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

The fixed branch is `MFUNC=1`, `CA=CB=0`, `YSCALE=1`, `CC=0`, `FISOKIN=0`,
`OFF=1`, `INLOC=0`, with failure/tension limits inactive. Rate/filter inputs are
omitted from the production API because they have no effect on this declared
experiment. Native qualification must explicitly supply inactive rate settings;
it must not rely on unresolved reader defaults.

`PrepareTabulatedShellPlasticity` validates a caller-owned immutable curve with
2–1024 points, zero initial plastic strain, strictly increasing abscissae and
positive nondecreasing stresses. It prepares the pinned elastic coefficients.
`UpdateTabulatedShellPlasticity` consumes accepted stress and accumulated
plastic strain, engineering strain increments and the element's actual
transverse shear modulus. It stages a result and publishes it only on success.
Pointers must refer to storage accessible to the executing host or device.
Prepared coefficients and curve values must remain immutable during use.

The interpolation cache is replaced by a fresh binary search with the same
strict knot-side choice. Extrapolation beyond the final supplied curve strain
is explicitly rejected, including a candidate that crosses it. The native
projection freezes the old-point curve slope within an increment; this code
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

`plastic_work_density` is the native MULAWC diagnostic
`0.5*(old_equivalent_stress + new_equivalent_stress)*delta_plastic_strain` in
J/m3. It is not total stress work or a guaranteed exact split into recoverable
and dissipated energy. Multiplication by section thickness weight and area,
and consistency with discrete resultant work, remain section responsibilities.

Focused local tests cover invalid input and unchanged output, a late
curve-domain failure with exact retry, and seven 64-increment cyclic sequences
plus one rejected sequence on both host and device. The separate native point
suite qualifies the material equations and original source curve. These tests
were written without a build/run; the integrating guarded run is required.
