# LAW90 solid18 selected force and history

This is a pure eight-point element value seam for the radiator's selected
LAW90 / engine solid17 / IINT2 / ICP0 / ISMSTR10 / JCVT1 / ISELECT1 profile.
It does not admit the source into an app model, ledger, resident participant,
or clock. The separate executed SDI gate now establishes actual blank-HU
IFLAG1, KCON20 GPa and raw-curve scaling; source/runtime admission remains separate.

`InitializeForce90` evaluates native TIME0 with original positions and the
supplied uniform velocity. Its history stamp is sample zero, not an interval.
`EvaluateForce90` requires the exact accepted reference, material and base
stamp and a positive dt. Each result stages all eight native point states,
the global reduction, original-slot negative internal forces and raw dt/STI
before publication. The HD scratch API leaves only unpublished scratch dirty
on failure; accepted input and publication destinations must be disjoint from
scratch. The host convenience calls publish only after success.

Material curves remain immutable caller-owned views. Their storage must outlive
every material/history and be valid in its execution address space. A copied
prepared value does not own or authenticate original source storage. Exact
source IDs, eight node slots, coordinates and density bits bind the reference;
all prepared scalar fields and the same curve backing bind the material.

## Selected native phases

The qualified reference/current geometry supplies actual center/point Jacobians,
72 PIJ values per point, selected B-I and quadratic-corrected engineering rates.
The caller doubles B-I tensor shear once, performs SRHO3 with actual accepted
rho and immutable storage volume, calls the complete LAW90 point, then uses
returned SSP in MQVISCB. SRHO3 uses PM1 (126), and HM_READ_MAT1548 defaults MAT_PARAM%rho toPM1;
MMAIN670/693 therefore uses PM1 in AMU=rho/rho0-1. PM89 is physical
source density and remains distinct from PM1. PM1 also controls point sound
speed; MQVISCB RHOREF is the actual current density for ISMSTR10. The direct
caller tests retain unequal PM1/PM89. The original element reference has one
density, so force admission explicitly requires its actual blank-RHOR default
PM1=PM89=reference density; no altered density is silently admitted. ET is the actual returned
IFLAG2 minimum slope/E0 or IFLAG1 MAX(1e20, slopes)/E0. It is never
replaced by a constant1. Pressure, dV, average volume and old/new Q enter
the no-EOS native work before storage normalization. The literal SI volume floor
is 1e-20 m3; sub-floor packet tests are not original working-unit parity claims.

The r/s/t visit order is preserved separately from stored IP=r+2s+4t. Running
length reduces before each point's material/viscosity phase. S8EFINT3 takes the
regular/selected-shear branch and skips cross-force terms. S8EFMOY3 reduces six
stress values, rho, EINT density, Q and EPSD; SRROTA3 returns source-slot forces.
No rotation inertia, pressure-center correction, material failure, J2 plastic
strain or plastic-work placeholder is added.

Complete pinned tag sources now close absence of PLA/WPLA: INI_MLAW_VARS 69/133
sets G/L_PLA=0; ZEROVARS_AUTO 81/83 and349/351 zeroes G/L_PLA and WPLA;
INITVARS_AUTO 179–184 and403–406 activates WPLA only for positive PLA.
HM_READ_MAT90 209–213 retains ten UVAR/three single-curve cursors and enables
G/L_EPSD. NativeTags executes those selected statements using the complete
MLAW_TAG_ definition; its six values feed the full native reducer.

## Independent oracle and packet layout

Every enclosing donor is SHA-256 and pinned Git-blob checked. New native
storage wrappers call the qualified independent native reference/current
routines, the complete SIGEPS90, S8ETOTSH10, S8EFINT3, S8EFMOY3 and SRROTA3;
selected SRHO3/MMAIN/MULAW/MQVISCB regions are extracted exactly. Source packet
unpacking contains no geometry/constitutive equations. Native recurrence owns
its own history and cursors; production history never feeds expected values.

The direct caller ABI has 37 binary64 values: ten UVAR, six stress, rho/EINT/Q/
EPSD (20 history values); six stress/SSP/EPSD/ET/VIS/OFF (11 point observations);
dV/averageV/AMU/work/rawDT/STI (six caller observations). Three integer cursors
travel separately. The force ABI has eight 40-value blocks (those37 plus
currentV/storageV/runningLength), ten global-history values, three diagnostics
(minimum rawDT/STI sum/work sum), and24 original-slot force values:357 doubles
plus24 cursors. All Fortran leading arrays retain native MVSIZ512 where required.

Tests compare same-unit values with the qualified point's2e-10 coefficient,
using stress/energy tensor scales, the average-volume scale for cancellation in
dV, and dimensional scalar scales for volume, length and dt. Independent
perturbation negatives cover histories, material/caller fields and forces.
No working-unit comparator is used for this SI force gate.

## Qualification boundary and commands

Author gates: CMake host configure/build and both CTest entries pass (six
numeric functions plus owning/source identity); two native C++ units and the CUDA-shaped
C++ unit pass syntax; full donor preparation/check passes. No native Fortran,
NVCC or GPU job was executed by the author. The explicit production sizes are
History8168 B, ForceTrial11840 B and ForceScratch26008 B. CUDA tests allocate
packet scratch explicitly and request no global stack limit.

Preserved evidence: `law90-solid18-force-host-{build,tests}-author-3.json` and
`law90-solid18-force-host-3.xml`; owning CMake run4 reports pass; the final PM1 correction is covered byrun5; earlier author1/2 logs preserve corrected test
coverage expectations (toy hardening/cursors), with unchanged production math.
Native/CUDA/source gates below are authored and require root execution.

```sh
cmake -S lib_utest/qualification/law90_solid18_force -B BUILD \
  -DTL_LAW90_SOLID18_FORCE_NATIVE=ON -DTL_LAW90_SOLID18_FORCE_CUDA=ON \
  -DTL_LAW90_RADIATOR_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-radiator-geometry-1 \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD -j1
ctest --test-dir BUILD --output-on-failure
```

Owning tests: `law90_solid18_force_host` (6), `law90_solid18_force_native` (5),
`law90_solid18_force_sourcenative` (1:all1345 constructors plus8 intervals),
`law90_solid18_force_cuda` (2:160-step independent recurrence with late-point
failure/retry; all1345 constructors plus2 intervals). Source/owning identities
include all reused native reference/point preparation and the authenticated
original binary fixture. Source gate emits12105 native packets; original CUDA
emits4035. No crash trajectory or app source admission follows from this gate.

Bazel owning targets: `//lib_src/materials/law90:caller`,
`//lib_src/elements/solid18/total_strain:law90_force`,
`//lib_utest/qualification/law90_solid18_force:host_check`.
Existing force/reference/point APIs and equations are unchanged. Their new
fixture targets only expose existing immutable test helpers; owning manifests
refresh only these reviewed build registrations. The separately frozen shared
force extraction57084e6 was root-qualified through old solid18 host/native/CUDA.

## Qualified explicit-HU1 and actual blank-HU extension

Root `law90-solid18-force-root-tests-2` passed the earlier complete explicit-HU1
force gate:6 host,5 native, all1345*(TIME0+8) source packets and2 actual CUDA
functions. Root fixes were harness/build only; production equations were unchanged.

The actual blank-HU extension leaves geometry/caller/force arithmetic unchanged
and consumes the extended point with IFLAG1. Source fixtures preserve all1345
EID/PID/node/coordinate/order records, explicitly using the executed native SDI
density bits for both reference and material. Raw original MPa ordinates with
YFAC1e6 retain native subtraction/scaling order. The old SI-ordinate scale1
explicit-HU1 controls remain separate and unchanged.

New controls add actual raw-curve material to the direct320-step caller, a
rotated/distorted160-step element trajectory, all1345*(TIME0+8) native packets,
and actual CUDA160-step rollback plus1345*(TIME0+2) source packets. The late
IFLAG1 witness checks evolving quasistatic energy, because its strain-norm path
slot correctly remains carried zero. Author7 host functions pass under
Release/NDEBUG. Final owning function counts are host7/native6/source2/CUDA4;
the new native/CUDA branch requires root execution. No new device scratch or
production state is added; all earlier sizes remain unchanged.

Author extension evidence: `law90-iflag1-host-author-1.json` passes all19
host functions across preparation/point/force; `law90-iflag1-identity-author-2.json`
authenticates all reused source owners. C++/CUDA-shaped syntax passes in
`law90-iflag1-syntax-author-{1,2,3}.json`; the first run preserved a test-helper
RecordProperty qualification error, corrected without production changes.
No native/Fortran/NVCC/device execution was performed by the author.
