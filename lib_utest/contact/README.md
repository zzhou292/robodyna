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
