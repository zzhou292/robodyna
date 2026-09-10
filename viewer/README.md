# Accepted-result replay

The viewer **builds and links against the isolated installed Chrono core+VSG package, and both bounded Vulkan replay gates passed on the RTX 5090**: 15 normal-contact frames and 81 elastic-coupon frames. Six core scene checks and three owning VSG CPU lifecycle tests passed. The [runtime checkpoint](../../crash-work/reports/vsg-r0-r1-runtime-checkpoint-1.json) records image/source hashes, actual device identity, visual review and resource measurements; [the build checkpoint](../../crash-work/reports/vsg-r0-build-checkpoint-2.json) retains compilation/link evidence. Dependency provenance is in [DEPENDENCIES.md](DEPENDENCIES.md). Run the viewer after the solver has exited, under the same workstation guard and lock. These are fixture replay gates, not full-vehicle or new mechanics qualification.

`robo_dyna_replay` uses the bounded [AcceptedReplay reader](../output/AcceptedReplay.h) and core-only [AcceptedReplayScene adapter](../chrono/AcceptedReplayScene.h). It never calls `DoStepDynamics`, creates a second FEA model, or derives stress from shape. The adapter owns fixed identity-frame visual carriers and copies accepted coordinates into one stable mutable mesh; VSG binds once and recomputes actual face normals during rendering. The normal-contact bundle supplies its real canonical wall. The elastic coupon has no wall.

The initial view uses one material per shape: blue moving geometry and a gray wireframe fixed wall when present. Wireframe keeps the patch visible during its small penalty-contact overlap while preserving every wall coordinate, triangle and transform; the overlay and capture manifest label this display choice. The view has a fixed camera, accepted time/epoch overlay, one loading worker, and a bounded 1280x720 window request. The coupon camera is close to a side view so its actual millimeter-scale travel can be inspected without scaling deformation. Camera bounds come from the reader's streaming validation pass over the complete moving trajectory; frames are not preloaded. The old contact bundle lacks full source/part mappings and is labelled a **nondeforming normal-contact rig**.

The guided plate uses a fixed oblique camera from incident −X, with Y up and equal X/Z sightline components. Its 0.2 m length lies along Y, its 0.1 m width along Z, and contact motion is along X. This view retains about 71% of both Z width and X travel in horizontal projection, exposing the surface and wall mesh without scaling displacement. It keeps the same trajectory-centered target, 40° vertical field of view and distance rule. The selected h/2 replay now captures all 201 frames through 200 ms and has a verified H.264 video. Its small elastic bend is visible; this fixture is not a large-crush demonstration. See [video evidence](VIDEO.md). The earlier near-edge-on capture and the externally interrupted first oblique attempt are preserved separately.

## Build and source checks

The root CMake options are `ROBO_DYNA_ENABLE_REPLAY` and `ROBO_DYNA_ENABLE_REPLAY_SCENE_CHECKS`. The scene checks require only Chrono core and GTest. The viewer additionally requires an isolated Chrono build with VSG enabled and the pinned dependencies; the exported target is `Chrono::Chrono_vsg` (`Chrono::vsg` is only the in-tree alias). No CUDA compiler, FEA module or mechanics qualification target is a viewer dependency.

The coordinator wraps each configure/build with `Total-Lagrangian-FEA/tools/run_bounded.py`, `crash-work/reports/workstation.lock`, one build job, at most two affinity CPUs, and the agreed RAM reserve. Root CMake adds the accepted-reader target before this viewer directory. Use the Vulkan/pkg-config environment documented in [DEPENDENCIES.md](DEPENDENCIES.md).

Six CPU scene tests passed against actual Chrono core, including the fixed-shape instance wireframe flag consumed by VSG ([XML](../../crash-work/reports/replay-vsg-scene-tests-2.xml)): stable mesh/shape handles, independent analytical normals after bending, immutable input/fixed wall, failed-frame preservation/retry, ownership/topology/order, float-coordinate collapse and exact initial/final horizon. Three owning Chrono VSG tests also passed: default and valid counts, invalid counts without mutation, and a deliberate callback failure proving configuration locks **before** any window/device creation ([XML](../../crash-work/reports/chrono-vsg-lifecycle-tests.xml)). The actual viewer checks one worker and post-initialize setter rejection. Those CPU cases and the actual window/PNG runs are separate evidence.

The local executable is `crash-work/build/robo-dyna-replay-r0/viewer/robo_dyna_replay`. It uses installed libraries under `crash-work/install/chrono-vsg-r0` and `crash-work/install/vsg-r0`; all runtime links resolved without changing system libraries or drivers.

## Interactive playback and indexed screenshots

After the coordinated build, the executable accepts:

```text
robo_dyna_replay BUNDLE [--capture NEW_DIR] [--fps 1..60] [--require-frames N] [--wireframe]
```

Without `--capture`, it displays successive recorded frames at ten frames per second by default, then holds the final frame. Pause, next-frame and close controls are provided. `--fps` changes presentation cadence only; the overlay always displays the actual saved simulation time. There is no interpolation, backward seek or deformation magnification in this slice.

The passing normal-rig gate uses all 15 frames in `crash-work/runs/robo-dyna-smoke-20260909`. The passing final coupon bundle is `crash-work/runs/elastic-coupon-b2-20260909`, with 81 accepted frames. The following are **inner commands** for separate guarded GPU runs into new directories, not instructions to overlap them with mechanics:

```bash
robo_dyna_replay crash-work/runs/robo-dyna-smoke-20260909 \
  --capture crash-work/renders/normal-rig-r0 --require-frames 15

robo_dyna_replay crash-work/runs/elastic-coupon-b2-20260909 \
  --capture crash-work/renders/elastic-coupon-replay-new --require-frames 81
```

The capture directory must be absent, have an existing parent, and lie outside the immutable input bundle. Chrono's capture routine reads the previous rendered swapchain image. One initial uncaptured render lets ImGui measure its new overlay window. For each accepted row, the viewer then renders that row without capture, holds its geometry and timestamp unchanged, and requests `frame-NNNNNN.png` before a second `Render()`. This includes initial frame zero and prevents a one-frame timestamp/geometry offset. `SetTargetRenderFPS(0)` prevents frame skipping. The viewer verifies nonzero output, decodes the PNG with existing vsgXchange, checks actual framebuffer dimensions, and records filename, accepted epoch/time, byte count and SHA-256 in `frames.csv`. It never overwrites a screenshot. Window closure or any rejected frame makes the run fail; a failure note may remain, but no success manifest is written.

`manifest.json` is written last, after every frame and the declared final horizon match. It records the source schema/scope, camera, dimensions, one-worker setting, two-render capture convention, image-index hash and that no dynamics ran in the viewer. Original owner identity is preserved; missing source/run/topology identities are not invented. Output is still visualization data, not a restart state.

The runtime gate must inspect the real window/images for visible geometry, clipping, actual bending and correct timestamps; successful PNG decoding alone is insufficient. Check the coupon's positive and negative tip states with the fixed camera, and retain workstation RAM/VRAM reports. First R0/R1 runs use at least an 8 GiB free-VRAM reserve and no concurrent mechanics workload. Offscreen/headless Vulkan remains unqualified. The subsequent offline coupon video passed its separate encoding, metadata, full-decode and visual checks described in [VIDEO.md](VIDEO.md).

The retained passing captures are [normal R0](../../crash-work/renders/normal-rig-r0-20260909-retry2/manifest.json) and [coupon R1](../../crash-work/renders/elastic-coupon-r1-20260909/manifest.json), both 1280x720. Independent checks matched all 96 PNG hashes/chunk CRCs and all accepted epochs/times to their original CSV rows. First/middle/final images were visually inspected by two reviewers. R0 completed in 13.53 s with 281 MiB peak sampled RSS and 125 MiB peak device-wide GPU growth; R1 in 62.864 s with 298 MiB and 135 MiB respectively. Each used two affinity CPUs, one loading worker, a 4 GiB RSS/growth cap and more than 30 GiB free VRAM throughout. R1 ended exactly at epoch 18830 / 0.1878596958391254 s; the physical tip movement remains unscaled.

The original R0 capture is preserved as a failed visual attempt despite successful PNG production: its first overlay was hidden, its fixed wall instance was solid, and the clamped light angle darkened the patch. The fixes use existing Chrono model/lighting APIs and ImGui's initial layout warmup. Chrono's worker API and three owning tests were committed locally as `96af26597b`; no renderer arithmetic or mechanics was changed for these presentation fixes.

The [R1 coupon MP4](../../crash-work/renders/elastic-coupon-video-r1-20260909/elastic-coupon.mp4)
contains exactly 81 recorded rows at 10 FPS: 8.1 s, 1280×720, H.264/yuv420p,
67,833 bytes, no audio. Its [separate manifest](../../crash-work/renders/elastic-coupon-video-r1-20260909/manifest.json)
retains every accepted-time mapping, encoder/package hashes, commands, probe
results and successful full error decode. Decoded frames 0/40/80 were inspected
independently: correct overlays, fixed camera and physical-scale coupon motion.
The original R0/R1 runtime checkpoint remains unchanged; its historical video
flag predates this encoding result. This is an elastic coupon video, with no wall
or vehicle-crash claim.
