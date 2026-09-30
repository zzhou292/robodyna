# LAW90 preparation and native curve values

This module implements the bounded single-curve preparation used by the
radiator LAW90 profiles. Point and element values have separate APIs and
qualification; this preparation value alone does not authenticate source or
admit a runtime material.

`PrepareSI` accepts explicitly named SI values at the native reader's
post-`HM_GET` boundary. The caller owns the immutable `CurveView` backing and
must retain it for every use of `PreparedMaterial`. Copying that value copies
borrowed addresses; it is not an owning source object or an authenticated
declaration. Outputs must be disjoint from borrowed immutable input storage.

The selected profile has one curve, nu0, Hys1, Shape1, Alpha1, TFLAG2, FAIL0,
and no rate/filter. Zero Shape/Alpha/reference density/curve scale/contact
modulus retain their native reader defaults. The resolved values are checked
after those defaults. Incoming Hys0 (including negative zero) selects IFLAG1
before normalizing Hys to1. Positive Hys1 selects IFLAG2; both classifiers are
preserved and supported. Other resolved hysteresis/shape/alpha values reject. A nonpositive tension cutoff uses the finite native
EP20 sentinel in the SI packet domain. Its unit-scaled equivalence to a
working-unit sentinel is not asserted; original TC15 MPa is far below it.

`ReaderValues` preserves the original card modulus, its initial UPARAM4 shear,
and separate contact PARMAT/UPARAM coefficients. `UpdatedValues` preserves
all FUNC_SLOPE outputs, updated E0 and the distinct Emax, PM20/22/24/32 values.
The original card21 MPa is therefore not silently replaced everywhere by the
curve's larger initial slope. The source converter/default and KCON target
unit receipt is now measured by the separate native SDI qualifier; app
admission still requires the original declaration and selected material identity.

`LookupCurve` returns unscaled VINTER2 values and an explicit zero-based native
cursor. SIGEPS90 applies the curve scale later in its own material stages.
Strict comparisons retain either adjacent slope at an exact knot depending
on the incoming cursor. Finite zero/negative interpolated values and endpoint
extrapolation are supported. There is no clamping of tension values before
the TFLAG2 material stage. Three independent cursors will belong to
each point's history; this helper does not own them.

The additive `materials/detail/Vinter2Value.h` retains the native loop counter
and incoming-cursor-dependent ILEN. Existing VINTER/LAW36/LAW44 APIs are
unchanged. Invalid shape, cursor, nonfinite intermediate/output or unsupported
profile returns without changing the caller's destination. Complete curve
shape is checked before interpolation; this is bounded value functionality,
not an optimized curve-pool lookup.

Source attribution and independent controls are owned by
`lib_utest/qualification/law90_preparation`. Donor pin:
`a62b27e6baa555d222a580d6218867d0be4d70b5`, OpenRadioss, AGPL-3.0-or-later.

The executed original MAT057 direct/export/re-read receipt selects IFLAG1 from
blank HU, with KCON20 GPa. Its curve retains raw source MPa ordinates and
`curve_scale_dimension=1e6`; native `FUNC_SLOPE` subtracts base ordinates before
scaling. The historical `CurveView::stress_pa` name denotes these unscaled
base ordinates, and `LookupCurve` returns unscaled values. Do not multiply an
already-Pa curve by1e6 again. The older explicit-HU1 fixture retains Pa ordinates
and scale1 as a separate qualified representation. Native HM_GET density
multiplications give771.9999999999999 kg/m3; canonical regrouped conversion gives
772. Reference and prepared material must use the same admitted density bits.
