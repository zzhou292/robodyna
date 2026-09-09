# Accepted coupon video evidence

The [elastic coupon MP4](../../crash-work/renders/elastic-coupon-video-r1-20260909/elastic-coupon.mp4)
is a presentation of the accepted B2 trajectory: **81 frames, 10 FPS, 8.1 s,
1280×720, H.264/yuv420p, 67,833 bytes and no audio**. Its SHA-256 is
`8611b74d19091412c444528bd5fc9c630408d89e94a8320900d16f16323037f0`.
The [video manifest](../../crash-work/renders/elastic-coupon-video-r1-20260909/manifest.json)
is a separate completed checkpoint. Older PNG/runtime manifests remain unchanged.

Every encoded frame maps to one previously checked
[R1 PNG row](../../crash-work/renders/elastic-coupon-r1-20260909/frames.csv)
and the original B2 `accepted-frames.csv`. Fresh checks matched all 81 PNG hashes,
owners, epochs and recorded times. Video timestamps use a fixed presentation
cadence; simulation time still comes from each accepted row and the overlay.
The last saved interval is shorter than the others, so the video does not assert
uniform physical-time sampling or a single exact playback scale. There is no
interpolation, deformation magnification, wall or new mechanics execution.

The [probe output](../../crash-work/renders/elastic-coupon-video-r1-20260909/ffprobe.json)
records one video stream, 81 container frames and 81 demuxed packets. The
[full decode](../../crash-work/reports/replay-coupon-video-decode-1.json)
used FFmpeg `-xerror` through the complete video and exited successfully.
[Representative decoding](../../crash-work/reports/replay-coupon-video-frames-1.json)
produced frames 0/40/80. Two reviewers inspected those decoded images: overlays
read 0 / 94.179263 / 187.859696 ms and epochs 0 / 9440 / 18830, with the same
fixed camera, visible blue coupon, changing physical-scale silhouette and no
clipping. Lossy encoded pixels remain presentation data, not a restart state.

The [encoding guard](../../crash-work/reports/replay-coupon-video-encode-1.json)
records the exact command: libx264, CRF 18, medium preset, yuv420p and fast-start
MP4, with no overwrite, input/filter/encoder worker limits of one, two affinity
CPUs, 2 GiB RSS cap and 60 s timeout. It completed in 0.754 s with 220.52 MiB
peak sampled RSS. Full decode completed in 0.252 s. Neither requested a GPU.

The executable package is Ubuntu amd64 FFmpeg **7:4.4.2-0ubuntu0.22.04.1**,
downloaded from its [official Ubuntu archive location](https://archive.ubuntu.com/ubuntu/pool/universe/f/ffmpeg/ffmpeg_4.4.2-0ubuntu0.22.04.1_amd64.deb).
The 1,695,740-byte package has SHA-256
`96beb71a9c3904c69a03c500caf9135370beb499e8599d6a544644cd9b909462`, independently
matched against local APT package metadata. It was extracted under
`crash-work/install/ffmpeg-r1`; system tools and drivers were not replaced.
Existing host shared libraries supply its runtime dependencies. The manifest
records both executable hashes, version/build-configuration output and the
package's copyright notice. The package identifies its normal binary build as
GPL v2 or later; this external encoder is not linked into the CAE application.

This completes the small coupon's R1 video evidence. It does not qualify
headless rendering, plate-wall contact, source-part rendering or the final
deforming vehicle crash. Those remain separate gates in
[RENDERING_ARCHITECTURE.md](../docs/RENDERING_ARCHITECTURE.md).
