# Rear-metal H8 reference qualification

Five host functions cover the explicit profile and old strict entry, common
geometry identity, eight mass terms on six source NIDs, initial orientation
reversal, signed-zero/ULP/topology rejection, aliased input and unchanged-output
retry. Two native functions reuse the already-qualified complete solid18 native
startup packet and add a second complete SMASS3 call with actual repeated NC
indices. Its rho and volume come from the native geometry packet, never the
production reference. Native nodal scatter distinguishes six physical nodes
from eight independent slots; the duplicate nodes receive two contributions.

The complete native geometry remains owned by `solid18_reference/native`.
This wrapper does not edit its donors, namespaces, equations or comparisons.
It adds an explicit interface and validated compact source-node association,
then calls the same original SMASS3 with ISROT0 and independent zero arrays.
Production owns per-slot mass; native node sums are qualification observations.

The root-exported original fixture is
`crash-work/reports/yaris-rear-metal-geometry-1`, manifest SHA
`e799eee2fb93e7189fb1bbc84a7d100c6e1f98e50d381dd71ce5d2c2d354fd4b`.
It contains 306 source solids / 476 nodes: PID2000016 has 210 (77 collapsed),
PID2000392 has 96 (32 collapsed). All 109 collapsed records preserve exactly
`[A,B,C,D,E,E,F,F]`; none become a compact six-node element. `prepare_fixture.py`
authenticates all twelve binaries and their source/local-node/SI-bit association,
then emits exact hex-float C++ only into the build directory. No durable second
population header is checked in; the TL build imports no app modules. The
separate root-only staging script reuses the existing frozen app scanner/API.

One original native function compares every cell in SI and original working
units using the existing unit-conditioned comparison and exact source/permutation
identity. Two tiny CUDA functions cover independent native values, actual mass
scatter, device aliased input, late rejection and retry; a third CUDA function
covers every original source cell. No constitutive or runtime result is implied.

Root gate (shared root resource guard):

```sh
cmake -S lib_utest/qualification/solid18_law44_reference -B BUILD_DIR \
  -DREAR18_NATIVE=ON -DREAR18_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=120 \
  -DREAR18_SOURCE_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-rear-metal-geometry-1
cmake --build BUILD_DIR -j2
ctest --test-dir BUILD_DIR --output-on-failure
```

Author host checks use native/CUDA options OFF and no original-fixture path,
1 CPU / 512 MiB. Native/Fortran/NVCC/GPU/original numerical execution remains
root-owned. The identity gate authenticates shared source extraction but does
not execute native equations. Existing affected native gates qualify the shared
reference extraction separately; new repeated-slot admission requires this gate.
