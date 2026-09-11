# Selected H8E solid18 reference and native nodal mass

This value gate covers the original 908 `Windshield_Adhesive` eight-node cells
(PID/SID/MID 2000977), using LAW36 / Isolid18 / JHBE17 / IINT2 / ICPRE2 /
ISMSTR2 / JCVT1. It supplies immutable startup geometry and translational
coefficients. It does not admit solid forces, owner/tie/contact participation,
accepted material history or a timestep certificate.

## Selected caller correction

The initial freeze `96909f4` qualified complete SDERI3B/SMASS3B leaves, but
misidentified them as the original H8E startup. Pinned `INITIA:1235-1264`
dispatches JHBE17 to **S8ZINIT3**, before fallback SINIT3. This changes the
selected geometry and nodal mass. The failed caller assumption, initial
passing leaf gates and the separate working-unit correction remain durable in
`crash-work/reports/solid18-selected-startup-diagnosis-1` and
`crash-work/reports/solid18-reference-unit-diagnosis-1`.

The corrected sequence is explicit:

1. Original-slot orientation correction and SRCOOR3 frame/local coordinates
   are unchanged. Source IDs and native-to-source permutation remain exact.
2. `S8ZDERIC3` computes center Jacobian, four higher coordinate modes, center
   volume and inverse center face scale. `S8EJACIP3` uses its actual binary64
   Gauss literal `.577350269189625D0`. `S8EDERI3` initializes eight real
   LBUF volumes and VOL0DP, retaining its native characteristic length.
3. Point storage is `IP=r+2*s+4*t`; caller visitation is r outermost, t
   innermost. Center volume and diagnostic sum of point volumes are separate.
4. For supplied virgin point density, `SVALUE0` accumulates
   `global_rho += (1 * point_volume / center_volume) * point_rho` in that
   visitation order. `SMASS3` assigns every source node
   `1 * global_rho * center_volume * (1/8)`. Thus masses are equal **within a
   cell**, even when point volumes differ. No shape-weighted nodal masses or
   rotational inertia are supplied by this selected branch.

The reference has no phantom point shapes or averaged derivative payload.
Its explicit fields are center Jacobian/modes, eight engine-indexed point
Jacobians/initial volumes, center/integrated volumes, characteristic length,
source-slot nodal masses and the actual initial global density. The latter
need not equal the supplied material density when point sum differs from
center volume. Failures stage no partial result; successful readback compares
named values rather than padding. This contained API correction precedes any
solid ledger/runtime participant.

At EID 2200907 the old leaf's eight masses range from
`3.6241855894704693e-05` to `3.6586838952581937e-05 kg`; the corrected host
selected producer gives eight equal `3.644683180660152e-05 kg` values.
Center volume is `2.7250378378542379e-07 m3`, point sum
`2.7249967705870301e-07 m3`, and native-order initial global density
`1069.9838747281588 kg/m3`. This example compares the old native leaf against
the corrected host producer; the new independent native caller gate remains
root-owned until its report passes. It is not a force-history qualification.

## Independent native and original source authority

`native/source-manifest.json` authenticates complete original files by bytes,
SHA256 and Git blob at OpenRadioss
`a62b27e6baa555d222a580d6218867d0be4d70b5`. Exact selected complete
subroutines S8ZDERIC3/S8EJACIP3/S8EDERI3 and SVALUE0/SCZERO3 are extracted
from their complete retained owning files; complete SMASS3 is compiled.
SRCOOR3 selected statements and complete orientation/frame leaves retain their
original paths/bytes. No equation, branch, loop bound or arithmetic is
instrumented. The wrapper supplies virgin density, zero stress/internal
energy, OFF/FILL1, and ordinary JALE/JEUL/JTHE/ISROT0. Dormant ALE/Q1NP context
and fail-fast MY_EXIT live in `NativeMassContext.F90`, reusable by the other
native solid startup owner. The full SMASS3 branches remain intact.

The same source fixture retains all 908 cells and 3672 nodes with source lines,
canonical indices, raw coordinate bits, once-converted SI, and authenticated
original cards/archive/canonical arrays. `verify_source.py` is unchanged.
Native SI and original t/mm/s calls are separate. SI comparison retains
`2e-11` relative plus `64 eps` of each dimensional group. Only converted
working-unit geometry uses the prior `256 eps * coordinate-conditioning`
allowance against a same-dimensional Jacobian/mode group. No global absolute
floor mixes units; source/slot/status identities remain exact. A physical
Jacobian/density change must not pass this roundoff allowance.

## Owning gates

Author: four host functions pass under one CPU / 512 MiB. Native source
preparation/identity and three C++ native test units pass light checks.
Native compilation/numerics, all-908 selected geometry/mass and actual CUDA
are root-owned and pending for this corrected producer. Receipts:
`crash-work/reports/solid18-selected-startup-author-1`.

The owning gate is 4 host + 3 native values + 2 native/source + 2 CUDA
functions and two source identity checks. It covers distorted unequal point
volumes with native uniform mass, center-versus-integrated volume, corrected
orientation and source slots, rigid placement, native bad-point rejection,
late mass overflow, failure preservation and aliased retry.

```sh
cmake -S lib_utest/qualification/solid18_reference -B BUILD_DIR \
  -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local \
  -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
  -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DTL_SOLID18_REFERENCE_ENABLE_NATIVE=ON \
  -DTL_SOLID18_REFERENCE_ENABLE_CUDA=ON
cmake --build BUILD_DIR -j1
ctest --test-dir BUILD_DIR --output-on-failure -R '^solid18_reference_'
```

Use root's serialized bounded native/GPU wrapper. Bazel owners remain
`//lib_src/elements/solid18:reference` and
`//lib_utest/qualification/solid18_reference:solid18_reference_host`.
The subsequent current-measure/force increment remains separate, with genuine
accepted eight-point density/pressure/volume/work histories and native ISELECT2
rather than a geometry-only force approximation.

Root selected-startup gate now passes all11 numerical functions and both
identities, including all908 original cells on native CPU and CUDA:
`solid18-selected-startup-root-tests-1`. The new independent native caller
therefore closes the corrected startup scope described above. Owning targets
pass in `solid18-solid24-owning-bazel-build-1`. The force/history and original
source-to-owner integration remain separate required milestones.
