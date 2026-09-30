# Native T3 reference slice

Source verified; **R1 startup, R2 prescribed geometry/rates and R3 prescribed
force/history pass** on 2026-09-09: seven startup, seven rates and eight force
standalone functions, plus two functions per stage on the authenticated six
source triangles (28 executions, including 10 new R3 functions).
This qualification directory complements the QEPH reference. It does not enter
the production solver, add a dynamics clock, or establish source ELFORM2/MAT024
equivalence. Three original physical nodes are retained.

The first configure/build and tests passed with the frozen numerical budgets:
[standalone report](../../../../../crash-work/reports/t3-r1-host-tests-1.json)
and [source-six report](../../../../../crash-work/reports/t3-r1-source-tests-1.json),
with their [standalone XML](../../../../../crash-work/reports/t3-r1-host-xml-1/)
and [source XML](../../../../../crash-work/reports/t3-r1-source-xml-1/).
Both runs used the shared workstation guard and no GPU. This qualifies only the
complete starter frame leaf and the explicitly selected source-expression
adapters below. The later R2/R3 evidence is recorded separately below. CUDA
and dynamics remain unexecuted for T3.

## Source ownership and closure

`source-manifest.json` pins OpenRadioss revision
`a62b27e6baa555d222a580d6218867d0be4d70b5`, 50 original files / 853,651 bytes,
and 17 complete routine extracts. The original Git blobs and SHA256 are checked
by `verify_sources.py`; the read-only recursive include check visits 42 files.
The eleven missing T3 leaves and `scr14_c.inc` were fetched serially at that
exact public revision after local Git/cache lookup. `stage_sources.py` limits
each request to20 seconds, each file to2MiB and inventory to16MiB. It verifies
the pinned Git blob before writing and refuses conflicting existing bytes.

The exact existing QEPH constants/modules, helper and LAW1 originals are
**borrowed by manifest path**, not copied or modified. Their license remains at
[`../qeph/original/LICENSE.md`](../qeph/original/LICENSE.md). All extracts retain
the donor notices. `EngineSkew.F`, `Cssp2a11.F` and `Sigeps01G.F` are also borrowed
unchanged from the qualified QEPH extracts. Original starter/material drivers
are retained context; their unrelated allocation/material branches are not
asserted to be a complete callable closure.

The selected complete engine leaf closure contains C3COOR3, engine C3EVEC3,
C3DERI3, C3COEF3, C3DEFO3, C3CURV3, C3STRA3, C3FINT3, C3FCUM3, C3MCUM3,
C3SROTO3, C3DT3 and C3UPDT3. The only literal helper calls in these complete
routine bodies are CLSKEW3 and CSSP2A11, both retained. Includes and the four
native modules are recursively pinned. The intended compiler profile is GNU
binary64, `MYREAL8`, `CPP_mach=CPP_p4linux964`, `COMP_GFORTRAN`, **no OpenMP**.
Obsolete architecture branches in the unchanged lock includes are context;
their `f_pthread` module/platform lock routines are not selected in this profile.
No engine bridge, history packet, engine execution or material-driver closure
is claimed by the startup gate.

## Callable startup boundary

`T3Reference.h/.cpp` owns the immutable `Reference` and staged `Initialize`.
`NativeT3Startup.F` owns two bounded native bridge calls:

| Output | Reference actually executed |
|---|---|
| Frame, area, local coordinates | **Complete native starter C3EVEC3**, without arithmetic edits. e1 follows node0→node1; e3 is the oriented facet normal; e2 is normalized separately. |
| Element mass, inertia and angle weights | **Selected C3INMAS source-expression adapter**, ordinary centered IGTYP1. No full starter allocation/material driver invocation. |
| Per-node mass and total inertia | **Selected SPMD_MSIN contribution expressions**, starting from zero element-local node sums. |
| Characteristic length and derivative slots | **Selected C3DERII expressions**, `L=2*A/max_edge`. The declared ISMSTR=-1 leaves startup PX1/PY1/PY2 zero. Engine current coefficients are not substituted. |
| Physical/added inertia partitions | Explicit diagnostics derived from the two terms of native XI; the native total is never reconstructed from the diagnostics. |

The native expressions are:

```
EM = rho*t*A
XI = EM*(A/NINE_OVER_2 + t*t*ONE_OVER_12)
p_i = ACOS(native law-of-cosines argument_i)/PI
m_i = 0 + EM*p_i
J_i = 0 + XI*p_i
```

All three ACOS expressions run separately; no clamp or renormalization repairs
their arguments/weights. Source `NINE_OVER_2`, `ONE_OVER_12` and `PI` come from
the exact borrowed `constant_mod.F`, including native constant folding. The
source-selected derivative slots are explicitly tested as zero. E/nu are
retained finite elastic experiment metadata, not consumed stiffness/material
behavior in this startup-only gate. No native startup timestep is reported.

The qualified standalone build compiles only the complete starter leaf, the small adapter and
the exact borrowed constant module. `prepare_sources.py` changes only the whole
`constant_mod` identifier token to `TL_T3_R1_CONSTANT_MOD` in prepared copies;
it writes source/prepared hashes and supports read-only `--check`. Both startup
Fortran units use the compile definition
`C3EVEC3=TL_T3_R1_START_C3EVEC3`. There is no QEPH native-library link or shared
COMMON block. A private C++ mutex serializes both calls because GNU Fortran may
retain local work arrays. Source arrays with MVSIZ129 and packets remain below
64KiB; there is no heap allocation in the element operation.

The standalone `CMakeLists.txt` owns `t3_r1_native` and `t3_startup_check`.
Its mandatory source-verification and prepared-receipt checks cover
`T3Reference.cpp`, prepared `NativeT3Startup.F`, `extracted/StarterC3evec3.F`
and the borrowed prepared constant module. It uses the qualified QEPH GNU FP
flags (`-fno-fast-math`, `-ffp-contract=off`, bounds checking, unlimited
fixed/free source line length), a private module directory and the same owning
native include search order. The app's opt-in `ROBO_DYNA_SOURCE_T3_STARTUP`
links that native library into the authenticated source fixture check.

## Frozen numerical domain and checks

These bounds were declared before numerical execution. Every world component
is finite with magnitude<=1e6m. The norm of each rounded binary64 edge difference
is in[1e-9,1e3]m, the minimum/maximum edge ratio is>=1e-6, and
`twice_area/max_edge^2>=1e-6`. Norm/area comparisons in the preflight use long
double; native arithmetic remains binary64. This keeps ordinary native squared
normalizations away from overflow/underflow. Starter C3EVEC3 has **no native
normalization floor**. The later engine C3DERI3 floor is outside this startup gate.

Before any ACOS, its three actual native arguments must satisfy
`abs(c)<=1-1e-12`. The returned frame must be proper/orthonormal within5e-13;
all mass/inertia partitions and L must be finite, positive and representable.
Domain, late underflow and nonfinite failures preserve every caller output byte.
Successful replacement does not assign semantics to C++ padding bytes.

Seven native host functions cover right/equilateral/scalene analytic weights,
area and mass; proper rotations/cyclic/reversed ordering and exact wall-edge-on
geometry; density/thickness/length scaling; shared physical nodes; malformed
inputs; ACOS/geometry boundary rejection; and late mass/inertia failure with
clean retry. The shared **test-only** `T3StartupTestOracle.h` uses independent
long-double world cross products and `atan2`, not the native ACOS calculation.
Its frozen comparison is `abs(error)<=2e-12*(dimension+abs(expected))`:
dimension1 for angles/frame, length or area for geometry, and each positive
mass/inertia partition's own magnitude. The threshold-neighborhood tests check
admission and preservation without asserting analytic precision at the cutoff.

Two separate app tests in `source_part_t3_startup_check.cpp` reuse the existing
authenticated `SourcePartContactFixture` for all six original triangles,
unmodified coordinates, IDs, converted density/thickness and raw fourth card
slot. They use the same independent oracle and assemble each contribution once
into the original117-node space. The tests explicitly report that original
MAT024 and full-part structural mass/dynamics have not been admitted.

## Resolved next-stage distinctions

The retrieved leaves resolve several boundaries for the later reference:

- Driver ISH3N2 sets IFRAM_OLD=1 and passes it to engine C3EVEC3's argument
  named ISH3NFRAM. Its CLSKEW3 branch is skipped, but the full leaf's helper is
  still retained. C3SROTO3 **copies** FOR5/MOM3 to its output at IFRAM_OLD1;
  it is not an omitted operation.
- C3COOR3 gathers X/V/VR for IRESP2. The explicit quarter-step velocity
  correction is in **C3DEFO3**, with both correction terms active for ISH3N2.
  QEPH CZCORC1 is not a replacement.
- C3DERI3 computes current half-edge coefficients and applies its own Y3 floor;
  the bounded reference must reject floor activation. C3CURV3's shear correction
  depends on geometry and all angular columns. C3STRA3 scales rates by DT1/AREA
  and accumulates eight components for ISMSTR=-1/ISTRAIN1.
- C3COEF3's shear coefficient requires an explicit GEO37/GEO38 branch; resolve
  that and IEPSDOT0/IRESP2 before the next adapter is frozen. C3FINT3 applies
  membrane/shear resultants with t and moment resultants with t². IDRIL0 adds
  no local drilling couple or QEPH hourglass history.
- The existing QEPH LAW1 wrapper embeds CZSTRA3 and QEPH packet types. Only the
  exact common post-strain constitutive leaves/expressions may be shared after
  component/work/history mapping is reviewed. C3DT3 uses DTFAC1(7), not index3.

These source findings define the later R3 branch qualified below; they do not
qualify T3 dynamics or other material branches.

## R2 prescribed geometry/rates: passed

The corrected native build passes all seven new rates functions and seven
unchanged startup regressions; the app build passes both rates functions and
both startup regressions on the six original source triangles. Evidence is the
[native report](../../../../../crash-work/reports/t3-r2-host-tests-1.json),
[source report](../../../../../crash-work/reports/t3-r2-source-tests-1.json),
[native XML](../../../../../crash-work/reports/t3-r2-host-xml-1/) and
[source XML](../../../../../crash-work/reports/t3-r2-source-xml-1/).
These are 14 standalone and four app executions, with nine new test functions
relative to R1. Numerical budgets and native arithmetic were unchanged.

The [first build](../../../../../crash-work/reports/t3-r2-build-1.json) failed
because the authored initialized Fortran derived-type declaration omitted
`::`; its later missing-member errors were consequences of that syntax error.
The authored/prepared files and reports were preserved before correction in
the [first-build checkpoint](../../../../../crash-work/checkpoints/t3-r2-first-build-failure-1/manifest.json).
The sole source correction inserted that declaration separator. The
[corrected build](../../../../../crash-work/reports/t3-r2-syntax-build-1.json)
and subsequent tests passed. No original donor file, equation or tolerance
changed, and no native numerical result is attributed to the failed build.

`T3Kinematics.h/.cpp` exposes `EvaluatePrescribed(reference, interval, output)`.
The reference remains immutable. The caller supplies three endpoint world
positions, three midpoint world velocities/angular velocities, base time, h and
sample index. No position, velocity, material history or clock is advanced.
`T3NativeGeometry.F` composes the **complete unchanged** C3COOR3, engine
C3EVEC3, C3DERI3, C3DEFO3 and C3CURV3 leaves. `NativeT3Kinematics.F` only binds
and packs that shared engine work. R3 will use the same actual work arrays.

The selected branch is ISH3N2/IFRAM_OLD1/IREP0/IDRAPE0/IGTYP1/ISMSTR-1,
IRESP2, explicit IMPL_S0/IMP_LR0, OFF/OFFG1. C3COEF3 and material/shear-factor
selection are outside this R2 operation. C3EVEC3's linked CLSKEW3 helper is
unselected at IFRAM_OLD1; material directions are checked unchanged. The engine
uses its own four native modules and renamed COMMON blocks, distinct from R1
and QEPH. A single `NativeEngineContext` mutex must cover all future T3 engine
calls. R1 geometry preflight/frame checks moved verbatim to `T3Geometry.cpp`;
the existing startup API, arithmetic, tests and prepared receipt are preserved.

Freeze these conditions **before the first R2 run**:

- Current positions obey the existing R1 coordinate, edge and normalized-area
  bounds. Before C3DERI3, the same native dot-product order must give
  `Y3>THIRTY2*EM15`; its native floor expression remains present but inactive.
  `EM15=ONE/EP15`, with EP15 formed from native integer products, is the actual
  binary64 constant. No wall projection criterion is used.
- Every velocity/angular component is finite. h is positive finite, h/4 must
  remain positive, and `base < base+h/2 < base+h` must be representable and
  finite. Sample index is nonzero. All native outputs are checked; late finite
  input overflow rejects without publishing any caller field.
- Active leaf arithmetic uses binary64 named ZERO/ONE/TWO/THREE/HALF/THIRD/
  FOURTH constants. Their integer literals are exactly representable;
  THIRD is native ONE/THREE and FOURTH is ONE/FOUR. No unsuffixed noninteger
  source literal is silently promoted/replaced. No fast math, contraction,
  OpenMP, floor repair or rescaled donor geometry is enabled.

Output material order is XX,YY,XY,YZ,ZX,KXX,KYY,KXY. The first five raw values
are m²/s; the last three are m/s. C3CURV3's nonuniform angular shear terms are
included. `raw_rate/AREA` is a diagnostic only. R3 must call C3STRA3's h/AREA
operation once on the raw private arrays, never reconstruct it from public
normalized rates. Corrected VX13/VX23/VY12 and current PX1/PY1/PY2 remain
distinct from the zero ISMSTR-1 startup derivative slots.

Seven standalone test functions use independent long-double affine gradients
and simplified local-triangle angular polynomials, all 18 input columns,
world covariance, cyclic local-order checks, exact edge-on geometry and late
failure/retry. Frozen arithmetic comparisons are
`2e-12*(dimensional_scale+abs(expected))`; geometry uses 1/L/L², raw membrane/
shear uses `L*V+L²*omega`, raw curvature uses `L*omega`, and normalized scales
divide by area. Common-world covariance uses `2e-11` with declared fixture
scales. Cutoff-neighborhood tests assert admission, not this accuracy throughout
the entire conditioning domain. A consistent unit Z-spin with h=.04/.02/.01
has normal-rate oracle `sin(h/2)-(h/2)*cos(h/2)^2`; the frozen halving ratio
range [.12,.13] tests its cubic residual, not a general second-order dynamics
claim. Finite-h instantaneous rigid velocities are not asserted exact nulls.

The private geometry work is 85*MVSIZ binary64 values (87,720 bytes at129),
with fixed bounded native scratch below1MiB. There are no element allocations.
`T3_R2_BUILD=ON` includes `T3Engine.cmake`, `t3_r2_native` and optionally
`t3_kinematics_check`. `prepare_sources.py --stage engine` prepares13 files
with four module identifier mappings and a checked separate receipt. Its
default startup mode still verifies the unchanged four-file R1 receipt.
Two new app functions in `source_part_t3_rates_check.cpp` use the same
authenticated source-six fixture and test-only input/oracle helpers. They do
not consume MAT024, source NIP3 or a physical history/timestep admission.

## R3 prescribed native force/history: passed

The corrected native build and all eight force functions pass, together with
the unchanged seven startup and seven rates functions. All six app functions
(two force, two startup and two rates) also pass using the original six source
triangles. Evidence is the
[native report](../../../../../crash-work/reports/t3-r3-host-tests-1.json),
[source report](../../../../../crash-work/reports/t3-r3-source-tests-1.json),
[native XML](../../../../../crash-work/reports/t3-r3-host-xml-1/) and
[source XML](../../../../../crash-work/reports/t3-r3-source-xml-1/).
Both numerical runs used the shared workstation guard, one CPU and no GPU.
The frozen equations, numerical budgets and source data were unchanged.

The [first build](../../../../../crash-work/reports/t3-r3-build-1.json) compiled
the native library, then failed to compile one test declaration that used one
`auto` deduction for byte arrays of different sizes. Before correction, the
affected native test, the analogous uncompiled app test and configure/build
reports were preserved in the
[first-build checkpoint](../../../../../crash-work/checkpoints/t3-r3-first-build-failure-1/manifest.json).
Only those test declarations were separated. The
[corrected native build](../../../../../crash-work/reports/t3-r3-fixture-build-1.json)
and [source build](../../../../../crash-work/reports/t3-r3-source-build-1.json)
passed. No numerical execution is attributed to the failed build.

The [R3 implementation contract](../../../../../planning/T3_NATIVE_FORCE_IMPLEMENTATION.md)
froze branch, history, units, numerical budgets and exit gates before the
first build/run. `T3ForceReference` returns a staged proposal using the same
immutable Reference, prescribed interval and private engine context. Its
26-value T3 history contains FOR/FOR_G/MOM/STRA/THK/EINT/EPSD/activity; no
QEPH stabilization state or fictitious viscous-energy partition is introduced.
The 92-double native result preserves all38 R2 observables before strain/time
diagnostic mutation, then26 history values,18 world force/couple values and10
diagnostics. Caller acceptance is a value copy, without a dynamics clock.

The private geometry wrapper now separates frame/derivatives from rates so
R3 inserts complete C3COEF3 in actual C3FORC3 source order. R2 still composes
the same operations unchanged. Complete C3STRA3, SIGEPS01G, C3DT3, C3SROTO3,
C3FINT3, C3FCUM3 and C3MCUM3 run under the fixed centered LAW1/ISH3N2 branch.
Small PM, EPSD and MULAWGLC work/DM adapters retain explicit source provenance;
they are not complete all-material-driver invocations. Complete C3UPDT3 is a
separate test-only signed-scatter bridge. Native positive internal forces are
subtracted from the nodal RHS, and C3DT3 index7 is a diagnostic only.

`T3_R2_BUILD=ON,T3_R3_BUILD=ON` registers `t3_r3_native` and optional
`t3_force_check` with eight standalone functions. The six authored
Fortran units and ten complete leaves have a separate17-file prepared receipt
(`prepare_sources.py --stage force`), borrowing SIGEPS01G/CSSP2A11 unchanged.
R1 startup default preparation remains four files, and R2 remains13 files.
All reached COMMON symbols use the same private T3_ENGINE prefix and mutex;
new force modules consume the R2 modules instead of recompiling duplicate
native constants/geometry. The operation's fixed native work/scratch budget
is below1MiB; this native gate establishes no CUDA resource or throughput claim.

Tests cover independent constitutive/rate/physical-moment and all18 fixed-
resultant virtual-power columns; load/hold/reversal; world covariance; native
drilling and finite-rigid distinctions; stiffness units and shared-node signed
scatter; complete malformed-history rejection; and late thickness/force
failure with retry. Two app functions add 48 source-six physical-mode checks,
history hold/reversal/retry and exactly-once native signed contributions in
the original117-node index space. This is a prescribed reference test, not a
resident full-part mechanics owner or source MAT024/NIP3 equivalence. The
existing14 standalone and four app startup/rates functions passed again after
the private-stage refactor. A future HD T3
port can reuse the already-qualified `ShellElasticLaw1` point helper; this
native oracle remains independent of that production implementation.
