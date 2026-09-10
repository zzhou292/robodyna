# Native tabulated plane-stress qualification

This test-only library compiles the unchanged pinned OpenRadioss `SIGEPS44C`
and `VINTER` routines behind a small C binding. It qualifies the experimental
TL-FEA tabulated shell point update; OpenRadioss is not a production solver or
runtime dependency. `source-manifest.json` pins the donor revision, bytes and
hashes, including shared native precision support already retained by the
QEPH qualification. Preparation changes symbol names only, to isolate Fortran
modules, COMMON blocks and routines from the existing native references.

The wrapper explicitly selects isotropic hardening, rate off (`CC=0`,
`ISRATE=0`), local plane stress, unit yield scaling, and no failure. It retains
the native virgin hardening slope, three evaluated Newton iterates, elastic
transverse shear and left-segment interpolation at exact curve knots. The
46-point SI fixture is original Yaris curve 2100270; its source declaration's
Cowper-Symonds C/P rate effects are deliberately inactive. This is not complete
MAT024 equivalence and does not resolve the source filter default. Both
wrappers reject plastic strain outside the supplied curve; failure, kinematic
hardening and nonlocal updates are outside the experiment.

`NativePoint.h` exposes native stress, accumulated/incremental plastic strain,
tangent ratio, total thickness strain increment, yield and sound speed. Its
plastic-work value is an independently evaluated endpoint-average von Mises
stress times plastic-strain increment, following the caller's `MULAWC`
diagnostic. It is not total stress work or a substitute for the section's
resultant-work ledger. The wrapper starts thickness at zero with unit layer
thickness to measure the increment without subtracting two physical thicknesses.

The five tests cover the analytical elastic limit; independent native and TL
persistent histories during original-curve load/unload with permanent strain;
an analytical perfect-plastic simple-shear yield/unload solution; every original
curve knot; and the actual native NIP3 section tables. Stress comparisons use
`1e-8 Pa + 2e-11 * max(abs(values))`; strain/ratio comparisons use
`2e-14 + 2e-11 * max(abs(values))`. Those bounds are fixed before execution.
The independent analytical checks have their own explicit tighter bounds.

`NativeSectionRule()` calls unchanged `COQINI` and `COQINI_WM`. It returns
positions `[-.5, 0, .5]`, membrane weights `[.25, .5, .25]`, and the native
moment weights. GNU Fortran uses `MYREAL8` without `-fdefault-real-8`, matching
the existing native qualification flags: bare `.0833333` literals therefore
round to default real before promotion. The resulting linear bending moment
differs slightly from exact `1/12`; the test preserves that donor behavior.
Native COMMON initialization requires serial test use.

The CMake directory can be included by the owning qualification build or
configured standalone after the production point-law patch is present.
It exports `law44_point_native` and, by default, the five-function
`law44_point_native_check` executable / `law44_point_native` CTest entry.
The root execution queue owns guarded configuration, compilation and testing.
No runtime or test result is claimed by this source patch.
