# Rear LAW44 H8 force gate

This packet qualifies the existing selected rear reference and 3D LAW44 point as an eight-point force/history recurrence. It retains both eight-distinct and original repeated-pair topology. Production reuse is limited to qualified current geometry/rate/frame and neutral density/viscosity/work utilities. LAW36 force/profile admission remains unchanged.

The independent oracle calls complete native geometry, S8EDEFO3, SIGEPS44/MSTRAIN_RATE/VINTER, S8EFINT3, S8EFMOY3 and SRROTA3 leaves already retained by their owning qualifications. New complete donors are DEGENES8, S8EDEFOC3 and S8ZFINTP3. The complete engine `ind_glob_k.F` is authenticated and only its complete INTAB routine is compiled. The exact selected S8EFORC3 +10 statement and MULAW default von-Mises plastic-work loop are extracted from authenticated complete bodies. Every wrapper that passes the large derived geometry packet has an explicit interface; native leaf arrays preserve their MVSIZ leading bounds and compile with bounds checks.

The native packet performs geometry in the supplied SI coordinates. The previously qualified LAW44 native point converts material values through the declared working units and returns SI values. The MQVISCB packet executes its existing native statements in the declared units, then converts Q, SSP-derived STI and DT to the public units. MULAW work is independently evaluated on SI stress/volume packets with the native volume floor converted explicitly. This is not a whole-solver native reduction-order or bitwise-unit-round-trip claim.

Native accepted state is carried independently across all intervals, never reset from the production result. The packet exposes 20 values and one cursor per point, 38 observations per point, 12 global history channels, 21 saved local coordinates, all 24 source-slot forces, complete 1,111-field current geometry, and eight diagnostics. This includes native storage-volume correction, AMU, actual SSP/ET, Q/DT/STI, mean pressure and work, so a force-only comparison cannot hide history errors. Same-dimensional comparisons use the existing solid18 gate's relative/operation-scale rule without changing production tolerances.

Author evidence: four host functions, C++ oracle syntax, CUDA-shaped syntax (launch syntax removed only in a temporary file), and source identity preparation. No author Fortran/native/NVCC/GPU or original numerical run. Root gates add three native fixture functions, one all-306 source function (three successive intervals each), two recurrent CUDA functions (96 intervals on both topologies, including late-point and phase rollback/retry), and one all-306 CUDA function. The original binary fixture preserves 306 cells, 476 nodes and 109 repeated-slot cells; generation reuses the existing authenticated reference fixture tool and does not import app production code.

Root execution under the workstation guard:

```sh
cmake -S lib_utest/qualification/solid18_law44_force -B <build> \
  -DCMAKE_BUILD_TYPE=Release -DREAR18_FORCE_NATIVE=ON -DREAR18_FORCE_CUDA=ON \
  -DREAR18_SOURCE_FIXTURE=/home/jsonzhou/Desktop/chrono-work/crash-work/reports/yaris-rear-metal-geometry-1
cmake --build <build> -j2
ctest --test-dir <build> --output-on-failure -j1
bazel test //lib_utest/qualification/solid18_law44_force:host_check
```

The existing reference and point qualifier BUILD files only expose their existing test values to this owning qualifier. Their numerical sources and native reference packet remain unchanged. Their prior source identity records are refreshed only for changed BUILD file bytes. Legacy shared force native/CUDA gates are already qualified after the separate shared-value extractions; this family slice adds no shared mechanics edits.
