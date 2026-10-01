# Operate the migrated product

Run commands from the `robodyna/` repository. Bazel is pinned to 9.2.0; this
workstation's executable is `../crash-work/tools/bazel-9.2.0-linux-x86_64`.
Use the enclosing workspace's bounded runner for builds and its shared lock.
The `run` and `render` commands own that guard internally: do not nest another
workstation-lock guard around them.

## Build and test

The following target/configuration combination has passed. Supply the declared
VSG SDK locations when building the full viewer package:

```sh
bazel build --config=cuda --config=sm120 \
  --repo_env=ROBODYNA_VSG_ROOT=/path/to/vsg/install \
  --repo_env=ROBODYNA_VSG_SYSROOT=/path/to/vulkan/xcb/sysroot/usr \
  //apps/cli:robodyna
```

Declare `CUDA_PATH`, `CUDACXX` and `CUDAToolkit_ROOT` consistently with the selected
CUDA installation. `sm120` is this workstation's qualified architecture; other
architecture profiles have separate qualification status. OpenSSL and VSG are
explicit local SDK dependencies, not copied libraries hidden in the source tree.
See `build_defs/README.md` and `build_defs/chrono/vsg/README.md` for their contracts.

The package targets share one CLI implementation:

| Target | Contents |
| --- | --- |
| `//apps/cli:robodyna` | CUDA runtime, native viewer and visualization assets |
| `//apps/cli:robodyna_headless` | CUDA runtime without viewer dependencies |
| `//apps/cli:robodyna_host` | Host validation/inspection tools |

Selecting a package does not change a global solver compilation flag. The old
`--define=robodyna_native_runtime=1` option is no longer needed. Python imports
are restricted to declared dependencies, avoiding accidental collisions with
imported repositories' general-purpose package names.

Focused suites include `//build_defs:bootstrap_host_tests`,
`//src/mechanics:neutral_tests`, `//tests/chrono:native_host_tests`,
`//src/simulation/driver:host_tests`, and
`//src/simulation/driver:native_request_test`. The exact full qualification command
is retained in the receipt linked by `QUALIFICATION.json`. Do not treat a root
wildcard build as qualification of every optional backend or SDK.

## Validate, run and inspect

```sh
./bazel-bin/apps/cli/robodyna validate case.json
./bazel-bin/apps/cli/robodyna plan case.json --resources resources.json --output new-plan
./bazel-bin/apps/cli/robodyna run case.json --resources resources.json --output new-run
./bazel-bin/apps/cli/robodyna inspect new-run
```

The output directory must be new and its parent must exist. `validate` verifies
bounded input schemas and source-file/member identities on the host. `plan`
performs native preparation, including CUDA work, without creating a physical run.
`run` executes the single qualified physical owner, closes its archive and reads
all saved samples through the C++ replay validator. `inspect` reports closed
metadata and its validation scope; it does not independently repeat full replay.

Use `src/simulation/driver/README.md` for the versioned case and resource schemas.
The current product execution profile is the qualified Yaris native V6 wall/self
case. This entry point is not yet a universal deck frontend for all retained
modules. Model archives and large outputs remain outside source control.

This workspace's repeatable short case and approved resources are in
`../crash-work/investigations/robodyna-yaris-101-inputs-1/`. Create new outputs;
never overwrite a qualification run or the original 100 ms archive. Changing a
resource JSON file does not authorize exceeding an operator's approved limit.

## Render accepted output

```sh
./bazel-bin/apps/cli/robodyna render new-run \
  --resources render-resources.json --tools render-tools.json \
  --output new-render --view front
```

A legacy accepted directory also needs `--producer-guard` naming its completed
producer receipt. The renderer replays stored states at scale 1, uses the declared
part palette, verifies PNGs and fully decodes the final MP4. It preserves input
inventories before and after the work. Numerical qualification and visual review
remain separate statements.

The viewer, ffmpeg/ffprobe and visualization assets are explicitly pinned by
`render-tools.json`. The native viewer and its file anchor can be resolved from
Bazel runfiles; no test executable is involved. See
`src/simulation/driver/rendering/README.md` for the small tool/resource schemas.
This workspace's exercised tool and resource manifests are in
`../crash-work/investigations/robodyna-render-inputs-1/`; rebuilding an executable
requires refreshing its tool pin before a new render.

An accepted visual archive is not a physical restart. Restart, complete energy
and load-path ledgers, broader input profiles and optional-module CUDA coverage
remain separate work items.
