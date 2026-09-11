# Selected mapped S6Z LAW42 force caller

This isolated value API advances one real six-node wedge material history and
all twelve native stabilization histories. It uses the qualified S6Z startup,
LAW42 total-strain caller and shared frame/mode helpers. Source cards, resident
ownership, contact and whole-vehicle admission are outside this target.

The explicit demo mapping is JHBE24/S6Z, total strain 10, ICP1, engine JCVT1
(public IFRAME2), IINT2, MATVIS1, IMAS0, ISORTH0, JLAG1 and IDTS6=0. The original
195 wedge cells and their original HG1 source provenance remain authenticated.
This is the selected HEPH/S6Z demo mapping, not original LS keyword-import parity.
The force entry requires positive dt, a matching immutable reference/material/
profile and exact caller base time/sample label. It publishes only after the
complete geometry, material, stabilization, force and history packet succeeds.
There is no independent solver clock or element-owner commit API.

## Native source and deliberate repair

`native/source-manifest.json` retains 45 complete source files, 711,197 bytes,
from OpenRadioss `a62b27e6baa555d222a580d6218867d0be4d70b5`. Git blob, SHA256,
byte count and the original license are checked before preparation. Runtime
`S6ZRCOOR3`, `SGCOOR3`, `S6ZDERITO3`, `S6ZDEFOT3`, `SZTORTH3`, `SORDEFT3`,
`S6ZDERI3`, `SDLEN3`, `S6ZDEFC3`, `S6ZDEFO3`, `S6ZFINT3`, `S6ZHOUR3` and
`S6ZRROTA3` execute independently of the C++ equations. Reference values and
source permutation come from the existing independent S6Z startup oracle;
material phases use `law42_solid_caller_native` and native HM_READ_MAT modulus.

The selected native wedge LAW42 branch has an uninitialized local `CXX`
(`s6zhourg3.F90:214,332–333,406`), although S6ZFORC3 supplies material-returned
SSP. The matching HEPH caller passes its initialized material sound speed into
SZHOUR3. This target declares `Law42MaterialSoundSpeedV1`, inserting exactly
`cxx(1:nel) = ssp(1:nel)` before the native CASE dispatch. Original and repaired
hashes and the insertion are retained. This is parity against that explicit
corrected donor; it is not parity against undefined unmodified execution.
The tests independently vary SSP and use zero DN to expose this damping term.

The only other native instrumentation copies already computed modal values,
G/FCL and intermediate/final EINT into observations; it never writes recurrence
inputs. The source manifest records all insertion anchors. `S6ZDERI3` also
computes an unused VZL from uninitialized auxiliary locals. S6ZFORC3 never reads
that output; this API and oracle intentionally do not expose or compare it.
The consumed volume, gradients and current geometry use the complete routine.
Dormant ALE, orthotropic and other-material routines stop if unexpectedly called.

For NPTTOT1, ALLOCBUF_AUTO aliases the local and global stress/density/energy/
volume/viscosity pointers. Therefore there is one accepted EINT. The material
update and both stabilization half-work terms advance it in native order.
The tests compare the first and final EINT as separate phase observations;
material work remains a distinct returned channel. FHOUR carries all twelve
values. The collapsed wedge makes the third modal velocity affine/null, so
virgin excitation alone cannot prove those three histories; a seeded hold
checks their retained values too.

Native working-volume/area floors are explicit SI-packet floors, as in the
shared LAW42 caller. The original mapped source fixtures lie well above them;
no equivalence for near-floor arbitrary working-unit conversions is claimed.
G0 is native PM22=SUM(Mu*alpha)=2*mu for this source, and SZETFAC multiplies it
by the returned ET factor before stabilization.

## Qualification

Author checks: five host functions, source fixture identity and native C++
syntax pass under one CPU/512 MiB. All native Fortran/CUDA/large source tests
are authored for the parent's serialized execution; they have not been run by
the author. The 195 source cells reuse the existing authenticated
`solid_common/source_fixture` owner rather than a copied geometry header.

Root gate (inside the normal heavy-job guard):

```sh
cmake -S lib_utest/qualification/solid6z_force -B BUILD_DIR \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_Fortran_COMPILER=/path/to/gfortran-local \
  -DTL_SOLID6Z_FORCE_NATIVE=ON -DTL_SOLID6Z_FORCE_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR --parallel 1
ctest --test-dir BUILD_DIR --output-on-failure
```

Numeric selectors: `solid6z_force_host` (5 functions),
`solid6z_force_native_test` (4 functions), `solid6z_force_source_test`
(all195 × 5 independent packets in 1 function), `solid6z_force_cuda`
(2 functions, each400 recurrent native/device packets). Native, caller,
reference and source fixture identities are separate CTest entries. Native
checks stop at the first mismatch. CUDA uses explicit bounded device scratch;
it does not request an oversized global device stack allocation.

The comparison uses a 3e-10 factor on each dimensionally homogeneous group,
including the six stress components as a tensor. Source IDs and permutations
are checked independently and cannot be absorbed by numeric tolerances.
Negative controls change a retained history, a half-work phase and a final
source-slot force. Rejected output preservation uses complete bytes; successful
history/output comparisons enumerate actual fields rather than object padding.

Owning Bazel targets:
`//lib_src/elements/solid6z:force` and
`//lib_utest/qualification/solid6z_force:solid6z_force_host`.
Affected shared/root targets: S6Z reference (frame/geometry), LAW42 point,
LAW42 solid caller and the tensor/mode/characteristic-length helpers. This
family increment changes no previously qualified material or native equation.
