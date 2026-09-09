# Native T3 reference slice

Source verified; **R1 startup passed seven standalone host functions and two
authenticated source-six app functions** on 2026-09-09.
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
adapters below. Engine geometry/rates, material force/history, CUDA and dynamics
remain unexecuted for T3.

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

These are source findings, not qualification of a T3 force or dynamics path.
