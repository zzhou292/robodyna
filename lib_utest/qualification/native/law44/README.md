# Native tabulated plane-stress qualification

This test-only library compiles the unchanged pinned OpenRadioss `SIGEPS44C`
and `VINTER` routines behind a small C binding. It qualifies the experimental
TL-FEA tabulated shell point update; OpenRadioss is not a production solver or
runtime dependency. `source-manifest.json` pins the donor revision, bytes and
hashes, including shared native precision support already retained by the
QEPH qualification. Preparation changes symbol names only, to isolate Fortran
modules, COMMON blocks and routines from the existing native references.

The wrapper explicitly selects isotropic hardening, local plane stress, unit
yield scaling, and no failure. Its default retains rate off (`CC=0`, `ISRATE=0`);
an explicit `RateInput` enables filtered total-rate VP2. It retains
the native virgin hardening slope, three evaluated Newton iterates, elastic
transverse shear and left-segment interpolation at exact curve knots. The
46-point SI fixture is original Yaris curve 2100270. The original five tests
isolate rate-independent behavior; the four rate tests enable the actual
C=8000/s, P=8 and resolved 10000 Hz cutoff. [RateReference.md](RateReference.md)
records the direct-import source closure and the additional tests. This is not
complete MAT024 equivalence. Both
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
`law44_point_native_check` executable / `law44_point_native` CTest entry, plus
`law44_rate_native_check` / `law44_rate_native` for the four rate functions.
The root execution queue owns guarded configuration, compilation and testing.
No runtime or test result is claimed by this source patch.

## Physical layer-thickness entry

`NativePhysicalThickness.h` adds `EvaluatePhysicalThickness` for the layered
recurrence oracle. It passes the actual layer-volume weight in metres and the
running reported thickness into complete SIGEPS44C. Native elastic and plastic
thickness additions retain their source order; the output reports thickness in
metres. The previous `Evaluate` API, `Result` layout and C symbol remain intact:
they call the same native body with THKLY=1 and initial THK=0.

Common validation/response decoding is factored into small host modules. No
production material/section function computes the reference. Rejection leaves
the public output unchanged, including when a finite native packet produces
vanished thickness. The selected domain still excludes failure and nonlocal
updates.

The two new host/native functions pass alongside both existing standalone CTest
targets. A 1728-interval source-rate load/hold/reverse path found 326 bit-level
thickness differences from the intentionally collapsed unit-increment shortcut;
its point and sequential thickness comparisons pass the existing budgets.
Filtered VP2 can continue native plastic flow during zero-strain hold as its
rate-scaled yield decreases. A first fixture incorrectly required constant PLA
there; that expectation was corrected without changing mechanics or tolerances.
The focused build used one CPU, 204236 KiB peak RSS and 3.63 s; no CUDA run.
