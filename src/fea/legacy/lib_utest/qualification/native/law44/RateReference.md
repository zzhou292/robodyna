# Source LAW44 rate and thickness qualification

The original Yaris part uses curve 2100270, C=8000/s, P=8, supplied VP=0,
blank LCSR, ELFORM2 and NIP3. At pinned OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`, the actual positive plain-curve
and positive-C/P branch selects LAW44. It precedes the generic LAW36 branch.
The starter resolves VP0 to VP2 and sets A=0 for the supplied curve, so the
whole tabulated yield and nonvirgin hardening slope receive the rate factor.
The special virgin slope remains E.

The source cutoff is **10000 Hz on this direct LS-DYNA import path**. This
conclusion does not assume that an omitted field adopts the CFG GUI default.
`cpp_lsd2rad_convertor.cpp:121,162-176` creates and installs the same converted
model without export or re-read. Its `ModelViewPO` objects hold only populated
attributes. A missing field returns unavailable, and
`cpp_get_floatv.cpp:201-205` explicitly returns zero for that case.
`hm_read_mat44.F:196-220` resolves ISMOOTH1 with zero Fcut to 10000 times the
source time-unit factor; that factor is one for this SI declaration. The
`Fcut=10E30` CFG default therefore does not determine this direct path.

[rate-source-evidence.json](rate-source-evidence.json) identifies all 14 source
files used for this conclusion by pinned upstream path, Git blob, SHA256,
byte count and role. Every cached file was independently checked against the
pinned Git tree before this evidence was written. They are audit references,
not extra production or compiled dependencies.

`NativeRate.F` uses the native PI constant and the selected caller expressions:

```
angular_cutoff = 2*pi*cutoff
alpha = min(1, angular_cutoff*dt)
filtered = alpha*total_shell_rate + (1-alpha)*accepted_filtered
factor = 1 + (filtered/C)^(1/P)
```

The recurrence and constitutive update execute in the unchanged `SIGEPS44C`.
The small C binding supplies C/P, the total shell rate, alpha and accepted
UVAR1 explicitly. The selected CZFORC3/C3FORC3 expression computes the scalar
rate from membrane and curvature increments and **accepted reported
thickness**, before CMAIN3 advances thickness. All three points receive that
scalar and retain their own filtered history. In the source ITHICK1 adapter,
the accepted thickness also sets this interval's point positions, force
coefficients and layer volumes. Original native nodal mass remains fixed.

The four additional tests are fixed before execution:

1. Actual-source filter rise and decay against both unchanged native UVAR1
   and a long-double closed recurrence, including the rate-scaled virgin yield.
2. A 1664-step original-curve load, hold and reverse path; native and production
   independently carry stress, plastic strain and filtered rate. Positive work,
   reverse yielding and native point fields are checked throughout.
3. The exact saturated-filter limit and equality with rate-off behavior when
   the supplied total rate and accepted filter are both zero.
4. A 1664-step three-layer membrane/bending path. Each implementation carries
   its own evolving thickness and three point histories. The test compares
   native scalar rate, stress, filter, separate WF/WM resultants, physical
   thickness factors, plastic diagnostics, weighted/minimum ETSE and last-point
   pre-update SIGY. The latter is the caller output consumed by CZFINTN1;
   MULAWC's local weighted `yld` is not copied back to caller `sigy`.

Stress, work and strain comparisons retain the original point suite's bounds;
filtered rates use `2e-11/s + 2e-11*max(abs(values))`. The closed recurrence
uses a separately fixed `2e-10/s` absolute bound. The native point binding
reports combined DEZZ; the production section preserves the actual two
thickness additions. Their section comparison uses the existing `2e-14 m`
absolute thickness budget, with no fitted correction.

Failure, curve extrapolation, kinematic/nonlocal plasticity and other MAT024
options remain outside this qualified branch. This library is a test reference;
TL-FEA owns the production CUDA mechanics. Root owns guarded builds and tests;
this source patch makes no runtime qualification claim.
