# Original Chrono demos through Robodyna

Verified 2026-10-01 from completed run, capture and encoder receipts. All three
retained examples ran for six simulated seconds through the native Robodyna
Bazel build. Their complete telemetry matched exactly between headless execution
and live capture. Every selected guard exited 0 with complete cleanup; captures
decoded every PNG and the encoders fully decoded the final MP4s.
Root's visual review passed for initial, intermediate and final images of all
three examples; its workspace receipt is
`crash-work/reports/robodyna-chrono-demos-visual-review-1.json`.

These examples use **CPU physics and Vulkan GPU rendering**. They demonstrate
retained functionality, not a CUDA MBD/SCM solver, experimental accuracy
validation, independent FEA/MBD build separation or physical restart.

| Example / Bazel target | Physical run | Capture | Delivered video |
| --- | --- | --- | --- |
| Spring / `//examples/mbd:spring` | 6,000 steps × 0.001 s | Every 40 steps; 151 states | 151 frames, 25 fps, 6.04 s |
| Rigid collisions / `//examples/mbd:collision_nsc` | 2,000 steps × 0.003 s | Every 20 steps; 101 states | 182 frames, 30 fps, 6.067 s |
| SCM rigid tire / `//examples/scm:rigid_tire` | 3,000 steps × 0.002 s | Every 20 steps; 151 states | 151 frames, 25 fps, 6.04 s |

The final recorded state is held in the video, explaining the small difference
between video duration and the six-second physical horizon. All videos are
1280×800, use actual simulated states at scale 1, and contain no interpolated
geometry.

## Observed behavior

- **Spring:** the original native and callback models agree exactly in recorded
  position, velocity, length and force diagnostics. Maximum displacement is
  2.70047 m; maximum position error against the continuous oscillator reference
  is 0.00526762 m. This reports the original discrete integrator's observed error;
  the analytical trajectory does not drive the model or rendering.
  The original unstepped spring line is not fully laid out in the initial frame;
  it appears normally after dynamics advances. That frame was retained.
- **Rigid collisions:** all 87 falling objects, five fixed walls and the mixer
  remain present. Seed `20260930` selects the original randomized setup. All
  2,000 stepped states report contacts, with a peak count of 617 and maximum
  observed displacement of 13.14696 m. The original Bullet/NSC/PSOR50 path is retained.
- **SCM:** the wheel travels 3.06909 m horizontally and turns 4.71239 rad.
  Contact is observed on 2,858 steps; peak reported contact force is 23,860.84 N.
  Maximum soil depression observed over the run is 0.217734 m. The model also records plastic
  sinkage history and a persistent rut; its 0.601228 m maximum plastic-sinkage
  state is a different quantity from current visible surface depression.
  GPU ray and force step counters are both zero for this CPU SCM profile.

The source physical setups, native timesteps and numerical methods are retained.
Bounded hooks add explicit horizons, source identity, capture cadence and read-only
telemetry. Headless execution constructs and attaches no renderer. Capture renders
the same state twice for screenshot readback without advancing physics again.

## Artifacts and evidence

The complete machine-readable record is
[examples/QUALIFICATION.json](../../examples/QUALIFICATION.json), including source
digests, closed-receipt hashes, timings, exact telemetry and encoder-reported movie
hashes. This documentation pass read small JSON receipts and checked movie sizes;
it did not rehash large PNG or MP4 payloads or rerun simulation.

Movies remain outside Git in the enclosing workspace:

- Spring: `crash-work/renders/chrono-demos-20261001-1/spring-video/movie.mp4`
- Rigid collisions: `crash-work/renders/chrono-demos-20261001-1/nsc-video-2/movie.mp4`
- SCM: `crash-work/renders/chrono-demos-20261001-1/scm-video/movie.mp4`

Headless results are under `crash-work/runs/chrono-demos-20261001-1/`:
`spring-headless-2`, `nsc-headless`, and `scm-headless`. Matching capture directories
are `spring-capture`, `nsc-capture`, and `scm-capture` under the render directory.
The complete telemetry objects are equal in every pair; run summaries differ
only in `headless` and `frames`.

The earlier failed spring headless attempt and first NSC encoding attempt remain
preserved with their original failure receipts. The selected NSC video is
`nsc-video-2`; partial older output is not promoted to successful delivery.

This example qualification is separate from the final repository-wide regression
gate and source-publication checkpoint.

## Reproduce

Use the SDK configuration and bounded build policy in the
[operator guide](../migration/OPERATING.md), selecting the three Bazel targets
listed above. Unlike product `run`/`render`, these demo executables do not install
their own watchdog. The following Bash example wraps a spring capture and encoding
with the existing guard. Run from the repository, choose a new output parent, and
retain the original qualified outputs.

```sh
demo_root="$PWD"
demo_out="$PWD/../crash-work/renders/chrono-demos-repeat-1"
mkdir "$demo_out"
demo_bound() {
  local demo_report="$1"
  shift
  /usr/bin/python3 -B "$demo_root/src/fea/legacy/tools/run_bounded.py" \
    --report "$demo_report" \
    --lock "$demo_root/../crash-work/reports/workstation.lock" \
    --cpus 2 --max-rss-gib 10 --min-available-gib 32 --timeout 1200 "$@"
}
demo_bound "$demo_out/spring-capture.guard.json" \
  --gpu 0 --min-gpu-free-gib 8 --max-gpu-growth-gib 6 -- \
  "$demo_root/bazel-bin/examples/mbd/spring" \
  --steps 6000 --capture-every 40 \
  --chrono-data "$demo_root/src/compatibility/chrono/data" \
  --output "$demo_out/spring-capture"
PYTHONPATH="$demo_root/src/compatibility/app" \
  demo_bound "$demo_out/spring-encode.guard.json" -- \
  /usr/bin/python3 -B -m viewer.video.encode \
  "$demo_out/spring-capture" "$demo_out/spring-video" \
  --ffmpeg "$demo_root/../crash-work/install/ffmpeg-r1/usr/bin/ffmpeg" \
  --samples-per-second 25 --output-fps 25
```

For collisionNSC, use `bazel-bin/examples/mbd/collision_nsc`, `--steps 2000`,
`--capture-every 20`, and encode at `--samples-per-second 16.666666666666668
--output-fps 30`. For SCM, use `bazel-bin/examples/scm/rigid_tire`, `--steps 3000`,
`--capture-every 20`, and encode at 25/25. Give every case its own new output names.
For headless verification, omit the three GPU guard options, append `--headless`
to the demo command, and use another new output directory. Headless output is
telemetry only and must not be sent to the video encoder.
