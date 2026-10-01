# Render accepted results

```sh
bazel run //apps/cli:robodyna -- render runs/yaris \
  --resources render-resources.json --tools render-tools.json \
  --output renders/yaris
```

The full target packages `//apps/physical_viewer:physical_replay` and its data;
it uses the same CLI source as the headless and host packages. No global build
selector is required. The parent output directory must exist. The output must be new and outside
the immutable accepted archive. By default the command produces overview and front
close-up views; use `--view front` or `--view overview` to select one.

For a coherent legacy accepted directory, supply its original producer guard:

```sh
robodyna render /path/to/accepted --producer-guard /path/to/producer.json \
  --resources render-resources.json --tools render-tools.json \
  --output /path/to/new-render
```

Normal product runs require their verified launch-result receipt, hash-bound
request/native/guard/output records and closed archive. Legacy runs retain the
existing guard/closure checks. Neither route runs GoogleTest or fabricates XML.
The older `viewer.postprocess` qualification pipeline is unchanged; its existing
test evidence retains its original scope. Coherent media output and numerical
qualification are separate claims.

The normal viewer opens the authenticated archive through `Replay::Open`, builds
the scene, publishes every recorded state and finishes capture only after all
PNG files decode. The existing encoder verifies captures, holds every saved state
at the requested playback cadence, uses explicit ffmpeg/ffprobe binaries, and
fully decodes its final MP4. No physics advances, geometry interpolation, extra
deformation scaling or substituted screenshots occur.

All heavy stages are serialized through the workstation lock: input inventory,
viewer capture, video encoding/verification and final input rehash. Both views
reuse the existing part-ID palette with seed 2 and deformation scale 1. The
final result reports `media_verified`, input immutability and `visual_review:
pending`; an operator still reviews framing and visibility. Failures retain
their directories and logs and never create a successful render result.

## Resource file

This example preserves the current workstation rendering policy. The 10 GiB
per-view allowance is PNG disk output, separate from RSS/GPU memory or the
simulation archive cap.

```json
{
  "schema": "robodyna.render_resources.v1",
  "cpu_threads": 2,
  "rss_bytes": 10737418240,
  "minimum_available_ram_bytes": 34359738368,
  "gpu_index": 0,
  "minimum_gpu_free_bytes": 8589934592,
  "maximum_gpu_growth_bytes": 8589934592,
  "timeout_s": 1200,
  "workstation_lock": "../reports/workstation.lock",
  "png_budget_bytes": 10737418240,
  "samples_per_second": 5,
  "output_fps": 30
}
```

The source viewer admits 2, 6 or 10 GiB PNG budgets and reserves 32 MiB per
saved state plus 4 MiB metadata. The 301-state delivery needs the 10 GiB choice.
Five saved states per second and 30 video frames per second produce 60.2 s of
slow-motion playback. Physical timestamps remain the original stored values.

This first presentation profile uses the qualified GPU0 viewer setup. The
inherited viewer has no portable explicit Vulkan-device selector; arbitrary
multi-GPU presentation is not qualified by this command.

## Tool file

`robodyna.render_tools.v1` requires `viewer`, `ffmpeg` and `ffprobe` objects,
each containing an explicit `path` and lowercase SHA-256. Paths resolve relative
to the tool file. `ffprobe` must be the pinned sibling used by the existing
encoder. A packaged tool may instead use `{ "runfile": "...", "sha256": "..." }`.
There is no PATH search, download or tool installation.

Optional `display_environment` overrides are restricted to `DISPLAY`, `XAUTHORITY`,
`XDG_RUNTIME_DIR`, `DBUS_SESSION_BUS_ADDRESS` and `VK_ICD_FILENAMES`. Otherwise the
current graphical session environment is inherited. Tool hashes are checked
before and after bounded stages. The qualified frozen renderer can be selected
explicitly for old deliveries; the full product package now also carries the
native viewer. Its logical executable runfile is
`_main/build_defs/app/viewer/physical_run/physical_replay`; the public Bazel alias
does not rename that output. Pin the actual built executable SHA-256 in the tool file.
The watchdog interpreter is independently capability-checked and recorded for
each stage. `--watchdog-python PATH` provides an explicit platform interpreter;
unsupported overrides reject instead of disabling the pidfd/waitid monitor.

## Native viewer data

For the native viewer, add `chrono_data` to the tool file. It contains an `anchor`
object with either a `path` or `runfile`, and a `files` array of
`{ "file": "relative/name", "bytes": 123, "sha256": "..." }` records.
The packaged anchor runfile is
`_main/src/compatibility/chrono/data/logo_chrono_alpha.png`.

The anchor is a file, so resolution does not depend on runfiles having a directory
entry. Its actual parent is the data directory passed as `--chrono-data` to the
existing C++ resolver. The manifest must cover `logo_chrono_alpha.png`, all files
under `vsg/` and `colormaps/`, including fonts and textures. Missing, unexpected,
changed, duplicated or escaping assets reject. Admission is bounded to 4,096 files
and 256 MiB. Other Chrono model-data directories are neither hashed nor copied.

The currently imported group has 61 files / 28,417,479 bytes. Store their actual
identities when preparing the pinned tool manifest. Asset hashes join executable
pins and are rechecked around rendering stages. Older frozen renderers without
the new option remain usable by omitting `chrono_data`; no unknown flag is added.

## Implementation ownership

- `config.py`: tools, resources, supported cameras and capture admission.
- `assets.py`: bounded visualization-data identity and file-anchor resolution.
- `inputs.py`: normal product and legacy accepted-input provenance.
- `stages.py`: shared watchdog execution and stage cleanup receipts.
- `worker.py`: existing encoder entry and capture-to-source correspondence.
- `integrity.py`: bounded, read-only whole accepted-directory inventory.
- `pipeline.py`: sequential stage composition and final product receipt.

These modules contain no rendering engine, material model, integrator or alternate
contact implementation. Host tests mock processes and use tiny files; they do
not execute a viewer, ffmpeg, GPU or physical simulation.
