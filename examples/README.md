# Building the retained examples

The current compile matrix is [BuildMatrix.json](BuildMatrix.json). It accounts
for311 original native demo main files plus the separate FMI template example,
26 retained CUDA programs,97 Python examples and17 C# examples. The preCICE YAML
application is listed separately. Multiple targets for spring, collision and
SCM capture/interactive variants do not inflate the source count.

These are different facts:

| Status | What the evidence establishes |
| --- | --- |
| Retained source | The original main remains present against its import-tree identity. |
| Declared target | Bazel expands a real public target containing the original main or its declared launcher input. |
| Compiled | The explicitly requested target completed under the named profile and closed resource guard. |
| Runtime qualified | The named simulation/import/transport actually ran and passed its own checks. |

Building an example does not start it. Most inherited interactive demos still
need explicit model data, output paths, display and resource admission. CUDA
availability, Python module import and a managed assembly build are not full
simulation results. See the named qualification records for actual videos and
physics checks. The historical `docs/verification/CHRONO_DEMOS_TESTS_INVENTORY.json`
is preserved as evidence rather than rewritten with current discoveries.

## Build one native program

From the unified repository root, the following builds the original CPU
slider-crank example using the existing workspace tools. It does not run it.
Use a new report filename for each attempt and keep the shared lock path.

```sh
rb_work="$(cd ../crash-work && pwd)"
python3 src/fea/legacy/tools/run_bounded.py \
  --report "$rb_work/reports/core-build-manual-1.json" \
  --lock "$rb_work/reports/workstation.lock" \
  --cpus 8 --max-rss-gib 16 --min-available-gib 32 --timeout 7200 -- \
  "$rb_work/tools/bazel-9.2.0-linux-x86_64" --batch \
  --output_user_root="$rb_work/build/robodyna-bazel-user-root" \
  --host_jvm_args=-Xmx2048m build --config=host --jobs=4 \
  --lockfile_mode=error --disk_cache="$rb_work/build/robodyna-action-cache" \
  //examples/core:build_system
```

The guarded runtime preset is described below. A complete CUDA build example is
in [the CUDA FEA instructions](fea/cuda/README.md). These paths refer to the
current enclosing workspace's tools and caches, not a standalone Chrono or
TL-FEA source repository.

## Discover the current graph

Use `//tools/verification/demo_matrix:demo_matrix`, or invoke the same module
from the repository root:

```sh
python3 -m tools.verification.demo_matrix.cli query-commands
```

Run those two read-only queries through the shared workstation guard, preserving
the full XML and separate public-label list. The capture helper accepts a JSON
request with `repository`, `bazel_prefix` (Bazel executable/startup options) and
`repository_options` (the declared SDK inputs). It executes both queries
sequentially, records output hashes and rejects build-metadata changes during
capture. It does not build or launch a simulation.

```sh
python3 -m tools.verification.demo_matrix.cli refresh \
  --repository . --query-xml /new-query/expanded.xml \
  --public-labels /new-query/public.txt --query-receipt /new-query/receipt.json \
  --output /new-query/inventory.json
python3 -m tools.verification.demo_matrix.cli status \
  --inventory /new-query/inventory.json --matrix examples/BuildMatrix.json
```

The parser reads Bazel's expanded graph, including source variables,
comprehensions, custom macros, private backend edges and public aliases. It does
not implement a partial Starlark interpreter. Missing examples remain visible;
new declarations do not retroactively turn old query snapshots into evidence.

## Plan and qualify compilation

Create a local operator environment JSON with schema
`robodyna.demo_operator_environment.v1`, absolute `repository`, `bazel`,
`output_user_root` and `workstation_lock` paths, `cuda_arch` (`sm75`, `sm86` or
`sm120`), `watchdog` (the existing `run_bounded.py`), `watchdog_python` (the
qualified platform Python), and `sdk_env` mapping the named SDK variables to their
admitted paths. Set `timeout_seconds` explicitly when the ordinary7200-second
per-batch build allowance is unsuitable.
Set optional `disk_cache` to the absolute path of the existing Bazel action
cache, such as the workspace's `crash-work/build/robodyna-action-cache` directory.
Both native-prerequisite and wrapper/demo builds pass this as `--disk_cache`,
preserving cached artifacts across configuration changes. The controller does
not implement or clean a separate cache. Changing this setting requires a fresh
controller root because resume retains the exact original environment.
Use the unified repository's `src/fea/legacy/tools/run_bounded.py` as `watchdog`;
no original standalone repository is needed. The CAD Python batch additionally
requires the admitted `ROBODYNA_PYTHONOCC_ROOT`; ROS node users require
`ROBODYNA_ROS_INTERFACES_ROOT` for the original custom message package.
`action_environment` may contain only the explicitly selected `CUDA_PATH`,
`CUDACXX` and `CUDAToolkit_ROOT` values. Keep workstation paths outside Git.
The provider still authenticates each SDK's detailed ABI/files; directory
existence alone is not SDK qualification.
Each batch still validates its required SDK subset. All SDK paths supplied in
the operator environment remain present in every query/build command, including
native prerequisites. This keeps local repository identities stable when Bazel
revisits cached discovered headers from a previous profile. Supplied paths do
not enable features or add dependency owners; the batch's typed profiles do that.
Plans record required and supplied SDK variable names separately.

This is the operator JSON structure, with example absolute paths. Replace the
paths and populate `sdk_env` with **all SDK variables required by the selected
batches**; an empty map below is a template, not a ready full-build environment.
`sdk_sets` in [BuildMatrix.json](BuildMatrix.json) lists the exact variable names,
and [DEMO_DEPENDENCIES.md](../docs/migration/DEMO_DEPENDENCIES.md) describes their
admitted installations. Keep this machine-specific file outside the repository.

```json
{
  "schema": "robodyna.demo_operator_environment.v1",
  "repository": "/work/robodyna",
  "bazel": "/work/crash-work/tools/bazel-9.2.0-linux-x86_64",
  "output_user_root": "/work/crash-work/build/robodyna-bazel-user-root",
  "disk_cache": "/work/crash-work/build/robodyna-action-cache",
  "workstation_lock": "/work/crash-work/reports/workstation.lock",
  "watchdog": "/work/robodyna/src/fea/legacy/tools/run_bounded.py",
  "watchdog_python": "/usr/bin/python3.10",
  "cuda_arch": "sm120",
  "timeout_seconds": 7200,
  "action_environment": {
    "CUDA_PATH": "/usr/local/cuda",
    "CUDACXX": "/usr/local/cuda/bin/nvcc",
    "CUDAToolkit_ROOT": "/usr/local/cuda"
  },
  "sdk_env": {}
}
```

Run every admitted batch with one command:

```sh
python3 -m tools.verification.demo_matrix.cli build-all \
  --matrix examples/BuildMatrix.json --environment /local/operator.json \
  --output /new-full-build
```

The controller captures the graph under the existing workstation guard, rejects
missing declarations/SDKs, and compiles batches sequentially. No simulation runs
automatically. Add `--batch native_multicore` to execute only that batch; its
result is explicitly marked selected-batch-only. The output root must be new
and outside the source repository.

Language batches have explicit `prerequisite_targets` for their shared native
libraries. The controller builds those at four compiler workers, then compiles
large generated wrappers and original assemblies at two workers, using the same
configuration and eight-CPU/16-GiB guard. Both phases have separate plans,
completed-target events and receipts. Prerequisite libraries do not increase demo
counts. This restores native-library parallelism after cache cleanup without
running more than two large wrapper compilations together.

Resume the same request after an interruption:

```sh
python3 -m tools.verification.demo_matrix.cli build-all \
  --matrix examples/BuildMatrix.json --environment /local/operator.json \
  --output /existing-full-build --resume
```

Resume rejects source, matrix, environment and selected-tool drift. It preserves
old receipts and reruns every explicit build through Bazel, normally using its
cache, so missing outputs are restored and SDK providers are re-admitted. It
never treats JSON alone as proof an executable still exists. Fingerprints include
dependency locks, CMake/shell/assembly inputs and shader source. Runtime archives,
media and binary model assets remain outside compilation qualification. Selected
Bazel/guard/interpreter/compiler identities are recorded; this is not a claim of
a hermetic operating-system runtime.

Read compilation evidence separately from discovery:

```sh
python3 -m tools.verification.demo_matrix.cli status \
  --inventory /existing-full-build/inventory.json \
  --matrix examples/BuildMatrix.json --build-root /existing-full-build
```

This verifies closed per-target receipts and reports their last successful
compilation, including batches completed while the controller is still running.
Prerequisites and unfinished batches are excluded. A later failed attempt does
not erase an earlier success or imply that the whole controller passed.
Use resume for current SDK/output re-admission. Failed attempts and
retries occupy distinct directories. Individual plans remain available for
reviewing a domain command:

```sh
python3 -m tools.verification.demo_matrix.cli plan \
  --inventory /new-query/inventory.json --matrix examples/BuildMatrix.json \
  --environment /local/operator.json --batch native_multicore \
  --run-directory /new-build --output /local/native-multicore-plan.json
```

The plan contains explicit public labels, required configurations, SDK paths and
the CPU/RAM guard requirements. No wildcard is sent to Bazel. Planning rejects
missing SDK inputs, omitted profiles, changed source/declaration snapshots and
existing output directories. Use `build-all --batch` for execution and its sealed
invocation/guard receipts. The command writes a Build Event Protocol log into its
create-only attempt directory.
Never execute build batches concurrently. The measured language-wrapper trial
uses two compiler workers; ordinary native batches and prerequisites use four.
The eight-CPU/16-GiB guard is unchanged. Recorded single-worker batch peaks were
5.130 GiB for NumPy/plot/CAD,4.884 GiB for baseline Python and3.655 GiB for the
largest managed run. These measurements justify the trial, not a guarantee for
every simultaneous pair. If the guard stops an affected batch, restore that
batch's `compiler_workers` to1 and start a fresh controller root; preserve its
failed receipt. Retained DEME preparation keeps its separate one-worker policy.

```sh
python3 -m tools.verification.demo_matrix.cli record-build \
  --plan /local/native-multicore-plan.json --guard /new-build/guard.json \
  --bep /new-build/build-events.jsonl --output /new-build/qualification.json
```

Recording requires the executor's sealed `invocation.json`, the exact planned
command, admitted resource limits and lock, successful guard cleanup and a
successful BEP completion for every explicit target. Skipped/incompatible targets
cannot pass. Old aggregate build receipts remain useful historical recipe
evidence; their members are not expanded through a newer graph to invent
individual current-target passes. Runtime qualification has separate receipts.

## Explicit native runtime

Direct native targets retain their original main functions and default paths:
data is normally `../data/` and output is normally `DEMO_OUTPUT/`. Merely running
a binary from an arbitrary directory does not resolve its model assets.

The first generic launcher admits one audited headless CPU preset, the original
slider-crank example. Build its target first, then launch it explicitly:

```sh
python3 -m tools.native_demos.run --preset core_build_system \
  --environment /local/operator.json \
  --executable /absolute/robodyna/bazel-bin/examples/core/build_system \
  --data-root /absolute/robodyna/src/compatibility/chrono/data \
  --output /new-native-case
```

It creates `/new-native-case/work` and the sibling `/new-native-case/data` link,
preserves the original arguments/settings, and uses the same workstation guard.
The preset audits that this main creates no renderer, CUDA context or peer
process and does not write into the data tree. Source/binary hashes, cwd, selected
data directory, guard and printed solver/slider telemetry are recorded. The data
directory identity does not claim full transitive asset hashing. Execution of the
real original example still needs its named runtime gate; filesystem tests alone
are not simulation evidence.

Other GUI/GPU demonstrations require separately admitted presets and resource
profiles. MPI and ROS have focused transport helpers and declared node launchers.
Full distributed simulations and paired preCICE/OpenFOAM runs require
case-specific peer, data and resource orchestration. The
generic path recipe does not claim to support demos with overridden/hardcoded
working-directory conventions, and build-all never invokes any runtime launcher.

## Profiles and ownership still matter

`--config=multiphysics`, `multicore`, `fsi-sph`, `sensor-optix`, `opencrg`, `yaml`,
`ros-sensor` and `ros-urdf` select actual implementations and shared declarations.
The current `yaml` configuration also enables global VSG. The native YAML batch
selects both `fsi-sph` and `fsi_tdpf`, compiling both actual FSI factory branches
for the five original native YAML mains and requiring the existing HDF5/HydroChrono
SDKs. The Python YAML batch retains its separately admitted narrower profile.
Do not replace these
with per-file macros or link separately configured mechanics copies together.
The matrix separates incompatible batches rather than omitting difficult cases
from an aggregate advertised as complete.

The full numerical backend remains a transitional composition: independent FEA
and MBD library closure is not yet complete. Canonical C++ Body/Mesh/System-family
definitions coexist with compatibility names and paths. Python/C# public names
and archive/factory identities remain intentionally compatible. Authenticated
SWIG declaration views, inverse source histories and per-profile wrapper tests
must remain intact during deeper API migration. Optional bindings share one
configured native implementation; a baseline proxy is not proof that every
optional ABI profile is qualified.

The26 retained CUDA examples include implicit ANCF/cuDSS and DEME paths. Their
successful build is separate from the explicit Yaris crash backend and from
their own GPU runtime qualification. Source-only discovery, compile status and
GPU coverage must remain separate as this matrix grows.

The current source-grounded [binding and module barriers](../docs/migration/BUILD_AND_BINDING_BARRIERS.md) record what example compilation does not yet establish.
