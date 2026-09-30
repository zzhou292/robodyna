# Focused surface-contact tests

This offline CMake harness builds the contact fixtures and existing utility
regressions from the same sources used by Bazel. It uses installed Eigen and
GoogleTest; it does not download dependencies or build DEME/the full solver.
CUDA tests fail if the GPU is unavailable. Disabling CUDA explicitly builds
only the CPU fixtures, which cannot qualify a GPU milestone.

From the workspace containing `Total-Lagrangian-FEA`, configure and build with:

```sh
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/configure.json -- cmake -S Total-Lagrangian-FEA/lib_utest/contact -B crash-work/build/contact-cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc -DCMAKE_CUDA_ARCHITECTURES=120
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/build.json -- cmake --build crash-work/build/contact-cmake --parallel 2
python3 Total-Lagrangian-FEA/tools/run_bounded.py --report crash-work/reports/tests.json --gpu 0 -- ctest --test-dir crash-work/build/contact-cmake --output-on-failure --parallel 1
```

Run the GPU command in an environment with access to the host NVIDIA device.
The initial workstation is an RTX 5090 (`sm_120`, CUDA 13.2). Override the
compiler and architecture explicitly on other machines.

The runner gives our process group at most two logical CPUs at lower scheduling
priority, limits common numerical-library thread pools to one, and serializes
jobs through the report directory's lock file. It preserves 32 GiB available
host RAM and 8 GiB GPU memory, stops at 8 GiB sampled process-group RSS or 4 GiB
total GPU-memory growth, and records observations in JSON. These are monitoring
limits, not hardware-enforced allocation quotas. Keep fixtures small: a large
allocation can occur between samples. Do not use this harness for a full car.

The CUDA runtime fixture tests device allocation, a kernel, synchronization and
readback. It does not establish an FE dynamics baseline. Surface-contact tests
cover only their implemented interpolation/contact law; they do not qualify
curved ANCF geometry, friction, CCD, or Yaris contact by implication.

Configure with `-DTL_CONTACT_ENABLE_FE_BASELINE=ON` to additionally build the two
existing ANCF3243 beam mass/reference regressions (`utest_3243`). This optional
baseline uses the unchanged FE kernels and reference CSVs. Its runtime-only
CUDA dependency has been separated from unused sparse/direct solver headers;
cuDSS is still required by the actual direct-solver consumers. Run just this
baseline with `ctest --test-dir crash-work/build/contact-cmake -R '^utest_3243$'
--output-on-failure --parallel 1` through the same GPU guard. It checks existing
mass assembly, not transient deformation or the selected conventional Yaris
shell formulation. The legacy determinant assertion is not a complete
positive-definiteness test.

The shared contact-mass and fixed-step stability utilities live in
`lib_src/collision/SurfaceContactMass.h` and
`lib_src/solvers/ExplicitStepStability.h`. Their 13 CPU and two CUDA checks pass
through this harness and Bazel. CPU expectations include independently
scattered nodal impulses, assembled noncommuting stiffness/damping matrices,
amplification eigenvalues, and long-double bound calculations. The CUDA tests
use one block and one thread, less than 4 KiB explicit result storage, and the
public physical-node force view to check one reset followed by additive loads.

This first mass adapter admits isotropic lumped translations and explicit fixed
nodes. It merges shared signed endpoint weights before squaring and rejects
unknown, rotational, consistent, ANCF, and offset-shell mass contracts. The
stability utility assembles conservative block-row bounds for frozen symmetric
positive-semidefinite stiffness/damping and the fixed-step velocity-first
update. Structural and contact contributions must share the same global rows;
independent per-contact timestep minima do not establish system stability.

These headers allocate nothing, own no accepted state, and require serialized
row accumulation. Build them without fast-math, reassociation, contraction, or
flush-to-zero; the test targets declare the required CPU/CUDA flags. Overflow,
positive-term underflow, stale attempts and minimum-step failure reject the
trial. After any failure, discard the complete force/bound assembly. A copied
step limit must pass `IsCurrentLimit` against current sealed scratch before
use. This utility is not a timestepper or a nonlinear shell/contact stability
qualification. Run just its targets with the existing guard and
`ctest --test-dir crash-work/build/contact-cmake -R '^utest_step_stability(_cuda)?$'
--output-on-failure --parallel 1`.

The next physical-node state target is `tl_explicit_nodal_state`, defined in
`lib_src/solvers/CMakeLists.txt` and the existing Bazel package. It has no test,
shell donor, DEME, cuDSS or Chrono dependency. The state owner and the separate
translation-step operation reuse the qualified nodal/mass/bounds contracts and
the preallocated accepted/trial publication pattern. The update order follows
Chrono's `ChTimestepperEulerSemiImplicit::Advance`, with checked trial state
instead of direct publication. Legacy ANCF state/solver clients keep their
existing interfaces.

Its first admission scope is at most 64 physical nodes, one fixed timestep,
explicit isotropic translational masses and fixed nodes. Contributors borrow
the returned stream/views and add force and complete stability contributions
after one reset. They must report every failure through the sticky assembly
status; omitted contributions cannot be inferred from the force array. Seal,
advance and commit are separate checked operations. Scratch forces are not an
accepted diagnostic history. External material/contact history participants,
rotational inertia, parallel scatter and nonlinear contact-switching safety
remain separate integration gates. `utest_nodal_step_cuda` exercises the public
owner and real force-driven updates. All 12 tests pass through CMake and Bazel,
including continuous oscillator/damped-motion refinement, an independent
discrete energy invariant, full-capacity free flight, late finite overflow,
clean retry and CUDA launch-error poisoning. The original failed test run is
retained: CUDA 13.2 reports `cudaErrorInvalidValue` for the zero-thread launch,
so the error assertion now admits that documented argument error as well as
`cudaErrorInvalidConfiguration`. Numerical expectations and runtime code were
unchanged. The workspace execution checkpoint retains the detailed evidence.
