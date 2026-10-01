# Native vehicle product driver

This module puts the qualified native V6 workflow behind a normal product entry
point. It reuses the existing source authority, `PreparedRun`, accepted archive,
replay validator and workstation watchdog. It does not invoke or link a GoogleTest
runner. The old qualification executable remains unchanged for comparison.

The new driver and Bazel graph require their own host/build and short-trajectory
qualification. Existing 100 ms results qualify the old pinned producer, not this
entry point. Source migration is not a new physical acceptance result.

## Operator commands

Host-only commands need no native executable:

```sh
bazel run //apps/cli:robodyna_host -- validate case.json --resources workstation.json
bazel run //apps/cli:robodyna_host -- inspect runs/yaris
bazel test //src/simulation/driver:host_tests
```

Build/package the native backend explicitly while that transition is being
qualified. These two commands use CUDA during original-source preparation, even
when `plan` creates no physical owner:

```sh
bazel run //apps/cli:robodyna_headless -- \
  plan case.json --resources workstation.json --output runs/yaris-plan
bazel run //apps/cli:robodyna_headless -- \
  run case.json --resources workstation.json --output runs/yaris
```

The full `//apps/cli:robodyna` target packages both the native runtime and viewer.
`robodyna_headless` packages the runtime without VSG; `robodyna_host` packages the
host tools only. All use the same CLI source and syntax. The former global
`--define=robodyna_native_runtime=1` selector is removed: choose a packaging target
without changing the configuration of every C++/CUDA dependency.

The output parent must exist and each launch directory must be new. `--backend`
and `--guard` permit explicit paths during packaging qualification. The normal
backend receives a hash-bound JSON request; no environment variables select its
physical controls. `run_bounded.py` still enforces process affinity and sampled
RAM/GPU limits; a numerical allocation forecast does not replace that guard.
The selected watchdog's actual directory is explicitly included in its child
Python import path, preserving sibling helper imports under Bazel safe-path mode
and explicit symlink overrides. The qualified watchdog sources are unchanged.
The CLI and watchdog may use different Python interpreters. The launcher probes
the unchanged monitor's actual pidfd/waitid preflight, trying the CLI interpreter
then `/usr/bin/python3`. `--watchdog-python PATH` selects an explicit interpreter
and rejects unsupported choices without fallback. Launch receipts record the
selected executable, version, hash and probe outcome. Incompatible `PYTHONHOME`
is removed for the platform watchdog; monitor semantics are never weakened.

`validate` checks schema and the named file/member identities. It does not inspect
all canonical arrays, validate physical source coverage or prove a stable timestep.
`plan` performs the existing source preparation and complete resource forecast.
`run` uses the same prepared object and physical clock, preserves cooperative
prefix closure, and performs C++ replay checks after releasing the live owner.
`inspect` reuses the current closed-summary/receipt/hash contract and labels its
metadata-only scope. Full all-frame C++ replay remains a separate stronger check.

For a legacy accepted directory:

```sh
robodyna inspect /path/to/accepted --guard /path/to/producer-guard.json
```

`render` reuses the normal physical viewer, archive reader and `viewer.video`
capture/encoding checks. It runs every capture and CPU integrity/encoding stage
through the same watchdog. See [render configuration](rendering/README.md).
The native VSG viewer and asset runfiles are packaged by the full target and
selected through explicit pins. No GoogleTest executable is required by
the product render command, and rendering does not confer numerical qualification.

## Case schema

`robodyna.case.v1` requires these top-level fields:

- `profile`: `yaris.native_v6.wall_self`.
- `canonical_directory`: path containing the prepared canonical `manifest.json`
  and its original arrays. Paths resolve relative to the case manifest.
- `files`: `source_archive`, `canonical_manifest`, `scope`, `declarations`,
  `glass_resolution`, `type13`, `wall_manifest`, and `solid_packets`. Each is
  `{ "path": "...", "bytes": 123, "sha256": "64 lowercase hex digits" }`.
- `original_members`: roles `member`, `auxiliary_member`, `original_wall_member`,
  `self_contact_combine_member`. Each records the exact original ZIP `member`
  name, `bytes` and `sha256`. Extraction preserves the authenticated original
  basename used by source provenance. ZIP directory components never select an
  output directory; duplicate basenames, traversal and ZIP symlinks reject.
- `run`: `duration_s`, `fixed_dt_s`, `samples`, `contact_activity`, `stage_timing`,
  `capture_qeph_rejection`, `verify_initial_retry`. Contact activity is explicitly
  `shell_removal` or `all_active_prefix`. Duration is at most 0.1 s; the existing
  one-million-interval native ceiling and owning physical checks remain in force.

The native profile preserves run/topology identity, source material/contact
controls and qualified capacity selection. It does not fabricate an initial gap,
change source geometry, add mass scaling or silently change the selected timestep.
Unknown/duplicate JSON fields, nonfinite numbers and boolean integer values reject.

## Resource schema

`robodyna.resources.v1` requires explicit values for `cpu_threads`, `rss_bytes`,
`minimum_available_ram_bytes`, `gpu_index`, `minimum_gpu_free_bytes`,
`maximum_gpu_growth_bytes`, `timeout_s`, `cooperative_maximum_elapsed_s`,
`stop_grace_s`, `archive_bytes`, `artifact_file_bytes`, `solid_worker_blocks`,
and `workstation_lock`. The cooperative elapsed limit must precede the hard
watchdog timeout. Solid workers admit 4, 8, 16 or 32 blocks through the existing
qualified helper. The native archive cap is 6 GiB, per-artifact cap 32 MiB.

The resource manifest records machine policy; editing it is not authorization to
exceed an operator's approved limits. The current workspace vehicle profile uses
four CPUs, 18 GiB RSS, 32 GiB available RAM, GPU0 with 8 GiB free reserve and 9 GiB
growth. The 100 ms baseline used 18 h cooperative / 19 h hard timeout, 30 s stop
grace, 6 GiB archive, 24 MiB artifacts and 32 solid blocks. Other GPU jobs remain
running. Reuse the workspace's existing workstation lock; never bypass it.

## Verification scope

`host_tests` exercises malformed or ambiguous input, source identity changes,
resource rejection, create-only output, failed guard propagation and environment
isolation with tiny transport fixtures. These fixtures are not physical cases.
`native_request_test` exercises independent C++ request parsing without accessing
source geometry or the GPU. It must be built and executed before qualifying that
parser.

`delivered_archive_metadata_test` is an explicit local integration gate. Set
`ROBODYNA_ACCEPTED_100MS` and `ROBODYNA_ACCEPTED_100MS_GUARD` to the preserved
accepted directory and producer guard. It verifies the real completed endpoint
and metadata-file immutability without reading all frame arrays or advancing
mechanics. It is not enabled implicitly by the ordinary host suite.

## Files and ownership

- `manifests.py`, `jsonio.py`: strict operator schemas and bounded JSON.
- `sources.py`: existing streaming hash utility plus bounded ZIP admission.
- `runtime.py`, `job.py`: declared tools, clean environment, request and guard.
- `receipts.py`: hash bindings for published product results.
- `inspection.py`: existing accepted-output metadata checks.
- `rendering/`: optional pinned viewer/encoder workflow and bounded input inventory.
- `native/Request.*`: independent normal-backend request admission.
- `native/Prepare.cpp`: source backing and qualified library preparation.
- `native/Execute.cpp`: common owner execution and closed-result replay.
- `native/Report.cpp`: operator forecast/report, never a physics implementation.

Production namespaces still reference pure compatibility helpers currently named
`test` in the imported code. Those helpers contain no GTest or environment reads
on this path. Move them behind permanent profile ownership only through a shared
refactor that retains old harness behavior; do not duplicate their settings.
