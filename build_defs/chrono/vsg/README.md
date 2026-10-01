# Native physical replay frontend

`//apps/physical_viewer:physical_replay` builds the existing accepted-archive
frontend from Robodyna-owned sources. The Chrono core and VSG implementations are
native Bazel libraries; **no previously built Chrono implementation is imported**.
The VSG module compiles eight original sources and the two STB sources selected
by its owning CMake configuration for shared vsgXchange.

An initial declared local SDK supplies external VSG 1.1.15, vsgXchange 1.1.12,
vsgImGui 0.7.0, glslang and Vulkan/XCB. The qualified workstation paths are:

```text
ROBODYNA_VSG_ROOT=/home/jsonzhou/Desktop/chrono-work/crash-work/install/vsg-r0
ROBODYNA_VSG_SYSROOT=/home/jsonzhou/Desktop/chrono-work/crash-work/dependencies/vsg-r0/ubuntu-dev/sysroot/usr
```

Pass these as Bazel `--repo_env` values. The SDK repository checks the version
profile and SONAMEs, records SHA-256/resolved paths/ELF dependencies in
`@vsg_sdk//:sdk.json`, and rejects old Chrono dependencies. Nothing is downloaded,
installed or copied into Git. The SDK still has its recorded installation RUNPATHs;
OS ABI/X11 libraries and an installed Vulkan driver/ICD remain runtime prerequisites.
Portable distribution of these external dependencies is separate qualification.

The core's current generated header remains the qualified host mechanics profile.
VSG is selected by its explicit Bazel dependency, not by claiming that every
inherited module macro describes every linked optional target. Neither current
core nor VSG implementation uses `CHRONO_VSG` to condition these sources. A unified
capability/configuration API remains platform migration work.

The frontend accepts `--chrono-data ASSET_DIR`. Its library receives the path as an
explicit context and does not know about Bazel or search the working directory.
The product launcher can resolve the runfile
`_main/src/compatibility/chrono/data/logo_chrono_alpha.png` and pass its parent.
`//build_defs/chrono/vsg:assets` includes VSG fonts/textures, colormaps and that logo.
Non-Bazel builds retain only their declared CMake data-directory fallback.

Source/geometry ownership and archive validation are unchanged: `ReadInput`,
`OpenSamples`, scene initialization and every saved accepted sample still pass
through the existing reader. `viewer_targets.json` records the separate native
application source port. Part-color compiler flags remain local to that source.

Before claiming readiness, the parent must run the SDK CPU test and asset tests,
build the real frontend, inspect dynamic library resolution, then perform a
guarded capture from a closed qualified archive with input-integrity and media
validation. These targets do not themselves establish successful rendering:

```text
//build_defs/chrono/vsg:sdk_test
//build_defs/app/viewer/tests:assets_test
//apps/physical_viewer:physical_replay
```
