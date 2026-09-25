# P1 normal response qualification

Source authored; no owning compile, numerical test or CUDA execution claimed yet.
Root owns all heavy execution. This does not admit full TYPE25 contact or a vehicle.

The independent native oracle compiles pinned I25FOR3 blocks and constant_mod
parameter declarations via native/prepare.py. Original files and actual Starter/
Engine-mode evidence have length/SHA pins. The generator preserves full history
positive/foreign storage branches, the selected non-adhesion spring branch, full
native damping conditional and damping-work loop. Wrapper inputs make IVIS2=1,
STIGLO=-1 explicit. It consumes already-prepared VN; it does not claim to test the
preceding vector interpolation or geometric penetration routine. Its zero-contact
assignments are the original no-thermal/no-prescribed-load branch. Only coefficient
observation assignments are inserted after native FF/C assignments; they cannot
feed native computation. Array scratch is initialized so unassigned public KT/CF
channels can be represented as invalid zeros. No production kernel is linked into
the Fortran oracle and no oracle is linked into production.

Host tests: complete288-case native corpus/both history layouts, initial maxima,
zero-contact state, EPP boundary/signed rebound, multi-step/retry/aliased history,
zero coefficients/floors, explicit engine modes, invalid/overflow atomicity,
physical unit scaling and the mm-to-m EPP boundary. CUDA groups compare complete
meaningful outputs against both native Fortran and the host implementation, change
launch shape/order and test failure preservation/retry. A few-ulp numerical
budget is fixed in Assertions.h before execution, with no unit-sized absolute
floor. Invalid channels are API-defined zero, not fabricated native observations.
No speed claim follows from these small correctness coupons.

Owning build (under the workspace build guard,4 compiler workers):

    cmake -S <TL>/lib_utest/qualification/radioss_type25_normal \
      -B <workspace>/crash-work/build/radioss-type25-normal-1 \
      -DCMAKE_BUILD_TYPE=Release -DTYPE25_NORMAL_CUDA=ON \
      -DCMAKE_Fortran_COMPILER=<workspace>/crash-work/tools/gfortran-11.4.0/gfortran-local \
      -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120
    cmake --build <build> --parallel 4

Run type25_normal_host/type25_normal_sources under a host guard; run
 type25_normal_cuda separately under the normal GPU0 guard. Expected source-authored
counts:13 host GTests,4 CUDA GTests (native corpus288 rows),1 source-preparation
CTest. Bazel header/source targets additionally verify pure production closure at
O0 and pinned extraction. They do not replace native/Fortran/CUDA CMake checks.

Preserve every failed receipt. Actual Engine controls were observed only for the
fresh no-target-scaling case; no original-CST or full integration claim is made.
