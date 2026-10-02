# Retained CUDA FE demos

These canonical Robodyna labels select all 26 existing first-party FE demo
executables. They alias their actual `cc_binary` owners through the temporary
in-tree repository boundary; no implementation is copied or compiled twice.
The original source names, notices, element formulations and per-target numeric
flags remain unchanged. This batch establishes build entry points; successful
compilation and guarded runtime qualification are separate evidence.

| Function | Build every demo in the group | Programs |
| --- | --- | ---: |
| Beam/shell sag and resolution | `//examples/fea/cuda/beams:all_demos` | 5 |
| Mesh deformation | `//examples/fea/cuda/deformation:all_demos` | 7 |
| Engineering joints | `//examples/fea/cuda/joints:all_demos` | 4 |
| Contact, stacking and drops | `//examples/fea/cuda/contact:all_demos` | 9 |
| Mixed elements | `//examples/fea/cuda/mixed:all_demos` | 1 |

Build the complete set explicitly under the workspace's existing guard:

```sh
bazel build --config=cuda --config=sm120 //examples/fea/cuda:all_demos
```

That is the target/configuration selection. For a complete guarded single-demo
command, run this from the unified repository root with the admitted SDK already
installed. Select the CUDA architecture appropriate to the qualified machine;
`sm120` is the current workspace profile. No simulation starts during this build.

```sh
rb_work="$(cd ../crash-work && pwd)"
export CUDA_PATH=/usr/local/cuda
export CUDACXX="$CUDA_PATH/bin/nvcc"
export CUDAToolkit_ROOT="$CUDA_PATH"
python3 src/fea/legacy/tools/run_bounded.py \
  --report "$rb_work/reports/cuda-beam-build-manual-1.json" \
  --lock "$rb_work/reports/workstation.lock" \
  --cpus 8 --max-rss-gib 16 --min-available-gib 32 --timeout 7200 -- \
  "$rb_work/tools/bazel-9.2.0-linux-x86_64" --batch \
  --output_user_root="$rb_work/build/robodyna-bazel-user-root" \
  --host_jvm_args=-Xmx2048m build --config=cuda --config=sm120 --jobs=4 \
  --lockfile_mode=error --disk_cache="$rb_work/build/robodyna-action-cache" \
  --repo_env=CUDA_PATH="$CUDA_PATH" \
  --repo_env=ROBODYNA_CUDA_MATH_ROOT="$rb_work/install/cuda-math-r0" \
  //examples/fea/cuda/beams:tetra10_adamw_sag
```

Use a fresh report filename for each attempt. For all 26 programs, prefer the
[`cuda_retained` controller batch](../../README.md), which admits the complete
SDK set and uses the required single-worker DEME preparation policy.

The aggregate includes every program, including all seven direct DEME consumers
and their cuDSS dependencies. It is a build aggregate, not a runnable simulation.
Individual aliases and aggregates carry `manual` tags so unrelated wildcard
builds do not unexpectedly select these expensive optional backend profiles.
Select the explicit aggregate above to compile the whole set.

The [target map](demo_targets.json) records all canonical labels, original binary
owners, source hashes, direct dependencies, host flags, link flags, data
attributes and literal input/output references. For example:

```text
//examples/fea/cuda/beams:tetra10_adamw_sag
//examples/fea/cuda/deformation:bunny_newton
//examples/fea/cuda/joints:double_pendulum_revolute
//examples/fea/cuda/contact:sphere_drop_hydroelastic
//examples/fea/cuda/mixed:mixed_elements_gravity
```

## Build dependencies and arithmetic

The retained implicit solver owners still use their original CUDA optimization
flags, including their existing fast-math policy. They are distinct from the
strict-arithmetic explicit Yaris backend. The aliases add no compiler/link flags
and do not replace either numerical policy.

All 26 existing dependency closures include Newton/cuDSS, even where the selected
runtime example uses AdamW or Nesterov. Reviewed BUILD overlays replace ambient
math-library search flags with the declared `@cuda_math` SDK. Its pinned NVIDIA
CUDA 13.2.2 libraries match the existing 13.2.86 compiler; cuDSS is 0.8.0.10.
The host toolkit still owns cudart and CCCL, so no second CUDA runtime is added.
See [SDK setup and admission](../../../../build_defs/sdk/CUDA_MATH_SDK.md).

The retained Newton source now uses a small cuDSS 0.8 descriptor adapter. It
preserves 32-bit CSR offsets/indices, double values, matrix policy, upper view,
zero-based indexing, column-major dense buffers and default reordering. The
original source hash remains pinned; a complete inverse proof authenticates the
five API substitutions. This is API compatibility, not a new solver algorithm.

Seven binaries depend directly on the retained DEM-Engine backend. `item_drop`
defaults to hydroelastic contact but compiles both selectable backends. DEME is
an optional dependency of these examples and is not the qualified vehicle's
native wall/self-contact implementation. Its foreign build and runtime helper
must be qualified under their existing resource limits.
Build `@dem_engine//:dem_engine` separately with one Bazel job before the complete native aggregate:
its inherited foreign CMake build uses four compiler workers. Running that foreign
action alongside four native compiler actions would exceed the worker allowance.

## Runtime data and outputs

Building does not run any GPU simulation. Twenty original binaries have no data
attribute; six declare the complete mesh filegroup. Many programs nevertheless
read relative `data/meshes/...` paths and write relative `output/...` paths.
The 26 complete literal mesh filenames inspected are present in the retained
source tree; other references are directories or parameterized filename prefixes.
That does not establish a portable runfiles layout or verify every parameter choice.

Do not assume `bazel run` from the product root supplies the old working-directory
layout. Runtime qualification must use an isolated output directory, correctly
staged mesh inputs and the existing CPU/RAM/GPU watchdog. Several examples have
fixed defaults or no command-line horizon; do not launch the entire set together.
No runtime parameter, timestep, precision setting or output behavior is changed
by these aliases.

## Completeness and qualification

The source audit accounts for all 26 first-party `main()` programs outside
`lib_utest`, tools and third-party dependencies, and verifies their bytes against
retained source commit `f0cdeffaef85ea1f97c2162790dbd091fb2e4853`. There is no separate
`lib_bin` CMake demo project to port. Unit-test/qualification drivers and the
third-party DEM-Engine's own examples are separate catalogs, not omitted members
of this first-party executable set.

`//examples/fea/cuda:catalog_test` checks one-to-one canonical mappings and complete
group/aggregate membership, including optional backends. It is a small host
metadata test; it neither proves CUDA compilation nor runs the listed physics.
All 26 programs passed guarded compilation in the `cuda_retained` batch of
`crash-work/investigations/robodyna-demo-matrix-full-2` in the enclosing workspace.
Their GPU trajectories remain separately unqualified by this build. The target
map retains its initial pending status as historical inventory; use the current
matrix's completed-target receipts for compilation evidence.
The map's `existing_*` fields retain the pre-overlay inventory as historical
evidence; current SDK edges are authenticated in
`build_defs/legacy/build_overlays.json`.
