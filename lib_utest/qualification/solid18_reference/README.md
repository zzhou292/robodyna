# Native solid18 reference and nodal mass

This value gate covers the original 908 `Windshield_Adhesive` eight-node cells
(PID/SID/MID 2000977), using the selected LAW36 / Isolid18 / JHBE17 / IINT2 /
ICPRE2 / ISMSTR2 / JCVT1 profile. It supplies immutable native geometry and
translational coefficients. It does not admit a solid force, owner, tie,
contact participant, accepted material history, or timestep certificate.

`lib_src/elements/solid18` owns six small headers: selected types and immutable
reference, orientation/frame, basis, startup geometry, and mass. Input and
returned mass retain original source slots. A separate native-to-source map
records the starter's negative-orientation top/bottom swap. Collapsed cells,
nonfinite arithmetic, repeated source NIDs, or nonpositive Gauss Jacobians
following that correction reject without modifying the caller's output.
An input alias into the old output is supported by staging the entire result.

The eight startup Gauss volumes, eight integrated nodal volume weights and
volume-averaged derivatives are distinct fields. `SMASS3B` gives each node
`rho * sum(detJ * H_node)`, with native total mass computed separately in
its original expression order. There is no invented rotational inertia.
Starter `BASISF` has its own point order and promoted default-REAL literals;
these slots must not be treated as the engine's eight material-history slots.

## Independent oracle and original source

`native/source-manifest.json` authenticates complete original leaves and
borrowed includes by byte count, SHA256 and Git blob SHA1 at OpenRadioss commit
`a62b27e6baa555d222a580d6218867d0be4d70b5`. Existing shared constant,
precision, hardware and element-layout donors are referenced in their owning
qualification directory. `prepare_sources.py` creates private namespaces and
exact selected `SRCOOR3` connectivity/gather/frame/projection extracts.
`SREPISO3`, `SORTHO3`, `BASISF`, `BASIS8`, `SDERI3B`, `SMASS3B`, and
`CHECKVOLUME_8N` remain complete native equation bodies.

The sole equation-adjacent instrumentation copies already-computed
`H/PX/PY/PZ/VLINC` from `SDERI3B` before accumulation to private observation
storage. The wrapper always uses one element. No expression, loop, branch or
array bound changes. The message context counts native bad-Jacobian reports;
the selected ordinary mass call supplies a minimal dormant ALE context with
`JALE=JEUL=JTHE=0`. No diagnostic stub computes geometry or mass.

`source_fixture` retains all 908 original cells and 3,672 original nodes,
including canonical indices, source lines, blank masks, raw coordinate bits
and once-converted SI coordinates. The manifest authenticates the original
archive/member, canonical arrays, complete part/section/material/curve cards,
and ordered selected raw rows. The source collector reused the existing
modelio bounded reader and field parser. It did not synthesize source IDs.
The header and manifest have fixed hashes in `verify_source.py`.

Each original cell is compared with an independent native SI call and with
native original t/mm/s arithmetic converted once at the output. Only that
second comparison uses a separate declared coordinate-conditioning allowance
(`256 eps * max(1, |world x| / native characteristic length)`). Derivative
components use that allowance times their same-dimensional derivative-group
norm, because a small component may be formed by cancellation of large
cofactor terms. All other numeric channels keep the original component-relative
conversion check, and dimensionless shapes are exact. The SI path
uses `2e-11` relative plus `64 eps` of the same dimensional component group
for cancellation near zero; no global absolute floor mixes units. Original
identity, permutation, successful admission, positive Jacobians and positive
individual masses are separate exact checks. This allowance cannot admit a
native rejected cell. No global additive bitwise reduction is promised.

## Owning gates and evidence

Author checks: four host functions and original-source identity pass under
one CPU / 512 MiB. Native donor preparation/identity and all three C++
native-test translation units pass source/syntax checks. Native numerical,
all-908 geometry and actual CUDA execution are root-owned and pending at
this first freeze. Author receipts are outside the repo at
`crash-work/reports/solid18-reference-author-1`.

The complete owning CMake gate adds three small native tests, one all-908
native/source test, two actual CUDA tests and native identity. It checks all
348 named values per reference, corrected orientation and original-slot
mass, distorted nonuniform weights, rigid placement, native bad-Jacobian
rejection, late coefficient failure, and preserved output followed by retry.
Raw object bytes are used only to verify no mutation on failure. Successful
comparisons and retries use named active fields, not padding.

Root configure, from the owning repository/worktree:

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

Apply the root's shared bounded build/GPU wrapper and workstation lock to
these commands. No other native or production targets are changed. New
Bazel owners are `//lib_src/elements/solid18:reference` and
`//lib_utest/qualification/solid18_reference:solid18_reference_host`.

The following current-measure/force increment remains separate: native
ISELECT2 depends on accepted stress/PLA and mutates reference volume and
stored internal energy before density/work. It must carry those histories
and the complete selected eight-point caller. This startup does not infer
them from its geometry, replace native selective derivatives with a generic
hex rule, or resolve remaining small-strain/viscosity switches by assumption.

## Working-unit comparison correction

Root's first complete gate passed the four host, three native value, two CUDA
(including all 908 original cells) and both identity checks. The native-source
SI comparison also passed for EID 2200907; only its original-mm-to-SI comparison
failed. Packed index89 is Gauss2 / native node6 / local-x derivative:

- TL and native SI: `0.0625683468958016 /m` (exact agreement).
- Native original mm arithmetic, converted once: `0.06256834689031404 /m`.
- Difference: `5.4875548549659925e-12 /m`; derivative-group scale:
  `167.70300125961126 /m`, hence difference/group `3.272186433009048e-14`.
- Native characteristic length `0.0039491571979098951 m`, maximum world
  coordinate `1.6992537000000001 m`, conditioning `430.28261850385087`.

The former component-relative conversion allowance was `3.913545609332551e-12`
for this cancelling component. `AgreeWorkingUnits` now applies the same
already-declared conditioning allowance to the derivative-group norm.
`Agree` and all production/native equations are unchanged. A focused original
cell test rejects a `1e-5 /m` physical derivative change and a one-ULP shape
change. The first failed report and complete single-cell dump are retained at
`crash-work/reports/solid18-reference-unit-diagnosis-1`; root reruns only the
two source-native functions for this test-only correction.
