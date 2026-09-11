# LAW36 solid point and caller qualification

The source fixture retains the complete original PID/SID/MID2000977 and curve
2100010 cards: 908 Windshield_Adhesive solids are the downstream source scope,
not an executable element fixture here. Eight curve samples retain their
original binary64 parse and once-only stress-to-Pa multiplication. SIGY on the
material card differs from the curve's initial yield and is not substituted.

Production is pure, allocation-free and has independent six-component history.
The oracle compiles complete SIGEPS36, MSTRAIN_RATE, VINTER and the unused
implicit link dependency M36ITER_IMP. Complete source files and exact hashes
are retained. native/prepare_sources.py only changes private names and takes
the recorded contiguous HM_READ_MAT36, MULAW and MMAIN extracts. It retains
separate returned DPLA and native rounded-PLA work. A fatal implicit stop is
linked explicitly and is never a successful result.

The original single-curve starter forces ISRAT0/FCUT0/VP0/ISMOOTH0. The native
default sentinel packet and a literal-zero default control run independently.
Native group IPLAS2 -> IPLA1, Iframe2 -> JCVT1, and ISTRAIN2 establish the selected
caller: MULAW alone increments total strain, while the later SSTRA3 call
requires ISTRAIN1. The point API uses supplied actual AMU/density/volume values;
no synthetic eight-node mass or ICPRE2 geometry is introduced.

The independent native caller receives SI packets, including MMAIN's literal
1e-20 storage-volume floor. Near-floor SI arithmetic is not evidence that the
original t/mm/s floor has been converted or is working-unit equivalent; see
the production module's explicit domain note. This gate does not derive the
original solids' quadrature volumes.

Author gate: five new host functions, three existing LAW44 preparation
functions, original source identity, and host syntax for the native C++ adapter.
Native and CUDA are root-owned gates; authored tests are not execution evidence.

Configure the owning CMake directory lib_utest/qualification/solid_law36_point:

    cmake -S lib_utest/qualification/solid_law36_point -B BUILD_DIR \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_Fortran_COMPILER=/home/jsonzhou/Desktop/chrono-work/crash-work/tools/gfortran-11.4.0/gfortran-local \
      -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc \
      -DCMAKE_CUDA_ARCHITECTURES=120 \
      -DSOLID_LAW36_NATIVE=ON -DSOLID_LAW36_CUDA=ON \
      -DSOLID_LAW36_LEGACY_NATIVE=ON
    cmake --build BUILD_DIR --parallel 1
    ctest --test-dir BUILD_DIR --output-on-failure

New selectors: solid_law36_host (5), solid_law36_shared_host (3),
solid_law36_native (4), solid_law36_cuda (2), and the two
solid_law36_{source,native}_identity checks. The opt-in legacy gate also
registers law44_point_native, law44_rate_native, law44_analytic_native.
These are the direct affected arithmetic callers of the extracted neutral
VINTER helper.

Owning Bazel targets:
//lib_src/materials:solid_law36_point,
//lib_src/materials:tabulated_shell_plasticity,
//lib_utest/qualification/solid_law36_point:solid_law36_host_check.
Only the material BUILD record in the T3 force manifest is updated; the previous
record is retained and no native donor or T3 equation is changed.
