# Rear-metal LAW44 point qualification

Seven new host functions and the five affected LAW36 host functions exercise
the closed original rear-material contract. Native/CUDA execution is owned
by the root lane. Source and C++ syntax checks do not claim native execution.

Native qualification compiles complete pinned SIGEPS44, MSTRAIN_RATE and all
VINTER donor routines with private namespaces, plus exact HM_READ_MAT44,
HM_READ_MAT and MULAW excerpts. The explicit SIGEPS44 interface is extracted
from the complete donor declaration block. The setup wrapper retains selected
raw source/default inputs and only supplies dimensionless curve scaling and
the warning callback; it checks the actual CA0 warning. Parameter calculation
does not consume production-prepared moduli, rate reciprocals, SSP or ET.

The independent packet runs in either original t/mm/s or direct SI, converting
the source curve, stress history and returned stress/sound-speed units. Its TF
packet has one unused leading pair, so native VARTMP is the retained zero-based
segment plus one. This is an explicit packet offset, not a new curve policy.
Native RHO/VOLUME/EINT arguments are unused in this IEOS0 leaf; AMU is supplied
independently. No density, energy or element arithmetic is inferred from them.

Four native functions cover original setup for both E values, independent
320-step histories in both unit systems, exact incoming cursor/knot/last
extrapolation, finite default-limit branches and converted stress floors.
Two actual CUDA functions advance all four material/unit combinations and
verify late failure/output preservation with a native retry from the same
accepted history. Tests retain the actual 46-point source curve and both
original PART/SECTION/MAT blocks in `source_fixture/source-receipt.json`.

Stress comparisons use the actual old-stress, G*increment and K*AMU operation
scale for cancellation. They do not use the possibly enormous native SIGY cap
as a stress-error allowance. Other channels use their own relative scale;
cursor, elastic/plastic branch and zero viscosity are exact. The host finite-cap
control rejects a 1 Pa stress corruption and a 1000x sound-speed unit error.
The 3e-11 comparison factor covers these fixed small recurrence fixtures; it
does not assert a universal large-strain error bound.

Root gate, from this worktree (use the shared root resource guard):

```sh
cmake -S lib_utest/qualification/solid_law44_point -B BUILD_DIR \
  -DSOLID_LAW44_NATIVE=ON -DSOLID_LAW44_CUDA=ON \
  -DCMAKE_CUDA_ARCHITECTURES=120
cmake --build BUILD_DIR -j2
ctest --test-dir BUILD_DIR --output-on-failure
```

Author host-only configure uses both options OFF and `-j1`. Author C++ syntax
checks cover NativeOracle.cpp, NativeTest.cpp and a temporary CUDA-shaped copy
with launch syntax/builtins removed; they do not substitute for NVCC execution.
`verify_source.py` authenticates all 28 donor records, the exact source curve,
selected declaration boundaries and retained production/harness identities.
Only the new native sources/wrappers build here; no H8 mechanics admission or
whole-model gate is implied.
