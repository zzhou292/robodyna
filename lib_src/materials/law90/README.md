# LAW90 preparation and native curve values

This module implements the bounded single-curve preparation used by the
original radiator LAW90 profile. It does not yet implement stress/history
recurrence, an element, source import, a material registry or runtime admission.

`PrepareSI` accepts explicitly named SI values at the native reader's
post-`HM_GET` boundary. The caller owns the immutable `CurveView` backing and
must retain it for every use of `PreparedMaterial`. Copying that value copies
borrowed addresses; it is not an owning source object or an authenticated
declaration. Outputs must be disjoint from borrowed immutable input storage.

The selected profile has one curve, nu0, Hys1, Shape1, Alpha1, TFLAG2, FAIL0,
and no rate/filter. Zero Shape/Alpha/reference density/curve scale/contact
modulus retain their native reader defaults. The resolved values are checked
after those defaults. In particular, Hys0 becomes Hys1 but selects a different
IFLAG and is rejected. A nonpositive tension cutoff uses the finite native
EP20 sentinel in the SI packet domain. Its unit-scaled equivalence to a
working-unit sentinel is not asserted; original TC15 MPa is far below it.

`ReaderValues` preserves the original card modulus, its initial UPARAM4 shear,
and separate contact PARMAT/UPARAM coefficients. `UpdatedValues` preserves
all FUNC_SLOPE outputs, updated E0 and the distinct Emax, PM20/22/24/32 values.
The original card21 MPa is therefore not silently replaced everywhere by the
curve's larger initial slope. The source converter/default and KCON target
unit receipt remains an app-admission obligation.

`LookupCurve` returns unscaled VINTER2 values and an explicit zero-based native
cursor. SIGEPS90 applies the curve scale later in its own material stages.
Strict comparisons retain either adjacent slope at an exact knot depending
on the incoming cursor. Finite zero/negative interpolated values and endpoint
extrapolation are supported. There is no clamping of tension values before
the future TFLAG2 material stage. Three independent cursors will belong to
each future point's history; this helper does not own them.

The additive `materials/detail/Vinter2Value.h` retains the native loop counter
and incoming-cursor-dependent ILEN. Existing VINTER/LAW36/LAW44 APIs are
unchanged. Invalid shape, cursor, nonfinite intermediate/output or unsupported
profile returns without changing the caller's destination. Complete curve
shape is checked before interpolation; this is bounded value functionality,
not an optimized curve-pool lookup.

Source attribution and independent controls are owned by
`lib_utest/qualification/law90_preparation`. Donor pin:
`a62b27e6baa555d222a580d6218867d0be4d70b5`, OpenRadioss, AGPL-3.0-or-later.
