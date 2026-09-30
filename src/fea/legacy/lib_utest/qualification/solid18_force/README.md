# Solid18 selected eight-point force recurrence

This gate extends the qualified selected `S8ZINIT3` reference with the active,
positive-Jacobian `S8EFORC3` LAW36 value recurrence. It keeps original adhesive
PID/SID/MID 2000977 and its no-rate table declaration. It does not initialize a
resident participant, assemble a vehicle, transfer tied coefficients or advance
an owner clock. No material failure or element deletion is admitted.

## Public values and phases

`solid18::InitializeHistory` creates eight genuine virgin material, density,
storage-volume and original-volume histories from the selected reference. The
global initial density is the native startup reduction, which need not equal
the input density on a distorted cell. The initial internal-force cache is not
evaluated by this constructor.

`PreparePrescribedHistory` is the explicit value-test seed entry. `EvaluateForce`
consumes endpoint world positions, carried midpoint world velocities and an
exact caller label `(base_time, dt>0, next_sample)`. Reference identities and
material parameters must match the accepted history. Curve arrays remain
borrowed immutable host/device storage; their backing identity is part of the
history scope and must outlive the history. All eight trial histories, global
reductions, geometry, source-slot RHS forces, and work publish together only
after success. Failure leaves the destination unchanged. There are no couples
or fabricated rotational inertia.

The result separates current Gauss volume, corrected storage volume, unchanged
original `VOL0DP`, native global EINT density, point WPLA, internal-work increment,
plastic-work increment, q, unscaled dt and raw STI. Raw STI precedes the native
`SCUMU3` nodal `2/8` scatter. The force is the native negative internal force.

## Selected caller order

The immutable reference retains the once-only native-to-source permutation.
Current geometry never reverses it. The force path uses the current native
frame, center geometry, eight Gauss derivatives, selective plane averages and
cross derivatives. Accepted global/point PLA determines FAC and selective nu.
Point storage `IP=r+2*s+4*t` is distinct from native r/s/t visitation
`0,4,2,6,1,5,3,7`; positive equal PLA ties therefore select the last visited point.

Each visited point updates the running native length, complete finite-strain
rates, selective SDV, stored volume and EINT before SRHO3 density. The actual
LAW36 point returns SSP and viscosity; the native oracle then calls MQVISCB,
native internal work, stored-energy conversion, S8EFINT3 and S8EFMOY3. Production
uses the already qualified LAW36 caller once; because this selected LAW36 has
constant prepared SSP and zero material viscosity, it computes q before that
shared material/work call. The independent native order tests this equivalence.

`IRESP=0` leaves original double `VOL0DP` unchanged. The selected volume reset
retains SDV even when its storage correction resets. Global EINT uses the
initial center volume, not the current volume or an invented average. All native
non-additive phases and point/source association remain explicit.

## Native authority and limits

All source is pinned to OpenRadioss
`a62b27e6baa555d222a580d6218867d0be4d70b5`. `native/source-manifest.json` retains
complete SHA256/Git-blob/byte identities for 54 donor, include and default
records, including the complete geometry/force leaves and borrowed LAW36 and
selected-startup owners. The AGPL license stays with the original reference
owner. `prepare_sources.py` changes names only and extracts the exact documented
caller line ranges; no donor equations, comparisons or loop bounds are changed.

The resolved branch is Isolid18/JHBE17, IINT2, 2×2×2, ICPRE2, ISMSTR2, JCVT1,
IR4R8=2, IRESP0, ISCAU0, ISMDISP0 and IMCONV1. The retained `hm_read_prop14`
receipt supplies QA=1.1/QB=.05/CNS1=CNS2=0; ordinary SRCOOR3 supplies VIS=VD2=0.
Complete `FREIMPL`, `IMP_INIT`, `machine2.inc`, r8 precision and `SZ_DT1` receipts
close those choices. No QEPH defaults are borrowed.

Negative-Jacobian/small-strain fallback, ALE, implicit, thermal, porosity,
initial stress, alternate material laws and owner startup force evaluation are
outside this gate. This entry uses SI values; native literal floors, including
MMAIN's storage-volume `1e-20`, retain the SI point convention. Near-floor
packets are not original t/mm/s equivalence evidence. Original adhesive volumes
are far above that floor. Source geometry and the curve reuse their existing
authenticated fixtures; no source coordinates are reconstructed from SI.

## Tests and execution

Four host functions cover genuine startup histories, cyclic yield/unload,
source/phase and late-point rejection with retry, and native selection order.
Five native functions compare independently advanced eight-point histories,
the 400-step rotated/unrotated path, selection, and wrong-history/phase controls.
The six gradient channels and hydrostatic compression receive separate native
packets. A sixth native function uses the first/final original source cells and original
row 89 (EID 2200907), 32 transitions each. This is not an all-908 force sweep.
Two actual CUDA functions compare all 400 states with that independent native
trajectory and repeat with a late-point/phase rejection and successful retry.

Comparison uses named active values, exact source/permutation/sample identity,
and `3e-11` relative plus `256 epsilon` of the same-dimensional channel group.
There is no absolute floor. Volume differences use the current-volume scale.
The native storage-factor diagnostic is the returned corrected/old volume
ratio; production records the actual local factor. Object padding is tested
only for preservation of a rejected output, never as a successful value ABI.

Root owns native/CUDA/Bazel execution. Example owning configuration:

```sh
cmake -S lib_utest/qualification/solid18_force -B BUILD_DIR \
  -DTL_SOLID18_FORCE_ENABLE_NATIVE=ON -DTL_SOLID18_FORCE_ENABLE_CUDA=ON \
  -DCMAKE_Fortran_COMPILER=FORTRAN_PATH -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR -j 1
ctest --test-dir BUILD_DIR --output-on-failure
```

CTest owns `solid18_force_host`, `solid18_force_native_test`,
`solid18_force_sourcenative_test`, `solid18_force_cuda`, and four source/native
identity entries (new force, selected reference, LAW36 and source fixture).
The host Bazel target is `//lib_utest/qualification/solid18_force:solid18_force_host`;
the production target is `//lib_src/elements/solid18:force`. Existing reference
and LAW36 host/native gates remain the directly affected dependencies. There is
no shared production arithmetic change in this increment.

Author evidence is limited to the four host functions, source identity and
preparation checks, and strict C++ syntax. Native and CUDA execution is pending
the root gate; this document does not claim those comparisons have passed.
