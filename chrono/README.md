# Reissner force-reference and CUDA operation checks

These checks reuse Chrono's existing Reissner4 shell and material operations.
They prescribe configurations and never advance Chrono dynamics. TL-FEA remains
the intended owner of CUDA state, force assembly and accepted time. The
application links actual owning libraries/sources, rather than a copied CPU
element formulation.

The retained 2026-09-09 results are **7/7 Chrono primitive tests, 12/12 connected
CPU force tests and 8/8 actual CUDA primitive tests**, all at their original
tolerances. Reports are `reissner-frame-tests-1.xml`,
`reissner-consistent-tests-2.xml` and `tl-reissner-frame-tests-1.xml` under
`crash-work/reports/`. The separate unmodified baseline reproduces its original
six passes and one failure, including every recorded metric. The TL header's
Bazel target also resolves successfully; its runtime tests are the CMake-linked
application tests because they require the actual Chrono reference library.

The isolated Chrono core at `crash-work/build/chrono-fea-consistent` enables
`CH_ENABLE_MODULE_FEA` and the default-off
`CH_REISSNER_CONSISTENT_FRAME_REFERENCE` option. Exact options are retained in
`crash-work/reports/chrono-fea-consistent-configuration.json`. Other optional
modules, demos, tests, OpenMP and SIMD are off. The ordinary core and original
FEA reference builds remain separate. Rebuilding this core requires one job,
a 2 GiB process-group RAM budget and a 900 s timeout under the workstation guard.

The coherent mode completes mean-frame contributions to the orientation and
spatial curvature derivatives, including ANS points with zero shape weights.
It restricts every pair of relative physical directors to less than 90 degrees;
common rigid rotations may be large. Caller force output is preserved through
failure. Internal element diagnostics are working storage, valid only after a
successful force evaluation. Nonzero stiffness/damping tangent requests fail
before changing caller output. The legacy mass routine remains unqualified.

From the workspace root, using that already-built core:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/reissner-consistent-configure-rerun.json --max-rss-gib 1 --timeout 45 -- cmake -S crash-app -B crash-work/build/reissner-consistent -DCRASH_ENABLE_CHRONO_REISSNER_CONSISTENT_CHECK=ON -DChrono_DIR="$PWD/crash-work/build/chrono-fea-consistent/cmake" -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/reissner-consistent-build-rerun.json --max-rss-gib 2 --timeout 120 -- cmake --build crash-work/build/reissner-consistent --target crash_chrono_reissner_reference_check --parallel 1
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/reissner-consistent-run-rerun.json --max-rss-gib 1 --timeout 45 -- crash-work/build/reissner-consistent/chrono/crash_chrono_reissner_reference_check --gtest_output=xml:crash-work/reports/reissner-consistent-tests-rerun.xml
```

The connected suite retains the original seven cases and tolerances. Five
additional cases cover noncoaxial bending/twist superposition; all 24 local
DOF force/energy derivatives before and after rotation; distinct initial node
frames with equivalent physical directors and body work; chart/late-arithmetic
failure and retry; and unqualified tangent rejection. Its admitted section is
one centered isotropic elastic layer on an initially rectangular Q4. It does
not establish warped initial geometry, plasticity, dynamics or Yaris ELFORM
equivalence. `ReissnerReferenceFixture.h` holds common setup and measurements;
the original and corrected-only test cases have separate source files.

`CRASH_ENABLE_REISSNER_FRAME_CHECK` separately builds the exact owning Chrono
helper and its seven unit tests. `CRASH_ENABLE_TL_REISSNER_FRAME_CHECK` builds
actual CUDA tests of `Total-Lagrangian-FEA/lib_src/elements/ReissnerFrame.h`
against the Chrono helper and independent finite differences. This is a frame
operation gate, not a complete CUDA element force or timestepper:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/tl-reissner-frame-configure-rerun.json --max-rss-gib 1 --timeout 45 -- cmake -S crash-app -B crash-work/build/tl-reissner-frame -DCRASH_ENABLE_TL_REISSNER_FRAME_CHECK=ON -DChrono_DIR="$PWD/crash-work/build/chrono-fea-consistent/cmake" -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120 -DCMAKE_BUILD_TYPE=RelWithDebInfo
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/tl-reissner-frame-build-rerun.json --max-rss-gib 2 --timeout 180 -- cmake --build crash-work/build/tl-reissner-frame --target crash_tl_reissner_frame_check --parallel 1
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/tl-reissner-frame-run-rerun.json --gpu 0 --max-gpu-growth-gib 1 --max-rss-gib 1 --timeout 45 -- crash-work/build/tl-reissner-frame/chrono/crash_tl_reissner_frame_check --gtest_output=xml:crash-work/reports/tl-reissner-frame-tests-rerun.xml
```

Build flags disable fast-math and fused CUDA multiplication for this gate. GPU
runtime errors fail the tests; there is no CPU fallback or skipped GPU pass.
All commands retain the shared workstation lock and reserve thresholds.
