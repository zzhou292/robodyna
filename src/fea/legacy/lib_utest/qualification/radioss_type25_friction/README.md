# Selected TYPE25 friction qualification

This owning gate composes the P1 normal oracle with exact native coefficient,
isotropic incremental force/history/work and row-phase blocks. It also reruns
normal regression tests after shared unit-conversion and precision wiring changes.
No native Fortran target is linked into production.

Source profile: MFROT2, IFQ10, IORTHFRIC0, INCONV1, INTTH0, ALPHA0=1, no part-specific
coefficient lookup in this packet, VISCFFRIC0. Actual first-entry flags and first-row
coefficient observations are pinned separately; they do not authenticate a full
vehicle population. Native per-secondary-row association and search remain outside
this packet qualification. Core and source design documentation is in
lib_src/collision/radioss_type25/FRICTION.md.

The generator imports the existing P1 source verifier and native oracle. New
Fortran wrappers preserve complete positive/foreign friction-history arithmetic
and the selected local I25IRTLM/I25MAINF phase predicates. Scratch that native code
does not assign in inactive cases is normalized to public zero values and marked
contact_active=false, never presented as a native observation. Phase-only copies
compare exact bits; coupled arithmetic histories use the same predeclared 64-epsilon
relative budget as their normal/tangential outputs, with no unit-sized floor.

The 75-row response corpus is 72 synthetic numerical packets, one promoted REAL*4
boundary normal, one zero-normal arithmetic packet and one exact actual
Yaris force-entry input. The latter pins the input observation, not a complete
I25FOR3 output or vehicle response. Coefficients are the separately pinned selected
Starter values. Relative velocity is formed with the original left-to-right
secondary-minus-four-weighted-main subtraction order. Original source IDs remain
fixture provenance and do not select production behavior.

Owning configuration, with root holding the workstation build lane:

    cmake -S <TL>/lib_utest/qualification/radioss_type25_friction \
      -B <workspace>/crash-work/build/radioss-type25-friction-1 \
      -DCMAKE_BUILD_TYPE=Release -DTYPE25_FRICTION_CUDA=ON \
      -DCMAKE_Fortran_COMPILER=<workspace>/crash-work/tools/gfortran-11.4.0/gfortran-local \
      -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120
    cmake --build <build> --parallel 4

Run host/source CTests separately from CUDA under their appropriate guards:

- type25_normal_host: 13 GTests; type25_normal_sources.
- type25_friction_host:21 GTests; type25_friction_sources.
- type25_friction_consumer_host: 1 GTest, production headers only, no manually repeated precision flags.
- type25_normal_cuda: 4 GTests.
- type25_friction_cuda: 5 GTests, including actual device multi-step row phases,
  stick/slip, changing normal, loss/recontact, SI and invalid/retry behavior.
- type25_friction_consumer_cuda: 1 GTest, actual FMA-sensitive inherited-precision check.

Expected total: 35 host GTests, 10 CUDA GTests and 2 source CTests. Confirm actual
executed counts and no skips; these are source-authored expectations until run.
Bazel friction consumer/source and normal header/source additionally verify header
closure and source wiring; they do not replace native/Fortran/CUDA qualification.
Preserve all failures and record actual binary/compiler/flags/source pins.
Matched whole-step and end-to-end GPU speed remain later mandatory promotion gates.
