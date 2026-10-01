# Chrono capture videos

```python
from viewer.video.capture import validate_capture
from viewer.video.encode import encode_capture

capture = validate_capture(capture_dir)
movie = encode_capture(capture_dir, output_dir, ffmpeg,
                       samples_per_second=5.0, output_fps=30)
```

Insert the `robo-dyna` directory in `sys.path`, or run from that directory:

```sh
python3 -B -m viewer.video.encode CAPTURE OUTPUT --ffmpeg /pinned/bin/ffmpeg \
  --samples-per-second 5 --output-fps 30
```

`validate_capture` returns an immutable `Capture`: `.directory`, `.metadata`,
`.frames`, `.manifest_sha256` and `.frame_index_sha256`. Each frame retains its
sample ordinal, epoch, physical time, filename, bytes and SHA256. Metadata is a
read-only nested mapping; `metadata_dict(capture)` returns a JSON-ready copy.
Both normal and recovered Chrono capture schemas are supported. The CSV and every
PNG are verified, including order, dimensions and strictly increasing physical
times and epochs. Capture completion is independent of simulation completion.
Recovered history availability and unknown horizon remain explicit.

`encode_capture` returns the absolute `Path` to `OUTPUT/movie.mp4`. The output
directory must not exist. The caller supplies its pinned executable; the sibling
`ffprobe` is used, and both tool hashes are recorded and rechecked. No system
tool discovery, installation or rendering occurs. Encoding and verification use
one thread. Run this CLI through the existing resource guard for a bounded job.

The presentation rate defaults to 4 saved states/s and 30 output frames/s.
Rates must be positive and the sample rate cannot exceed the output rate. Frame
boundary `i` is `ceil(i * output_fps / samples_per_second)`, so all states appear,
including a final hold. At 5/30 the holds are six frames each; at 4/30 they
alternate eight and seven frames. Repeated PNG bytes enter image2pipe at the
output rate. H264/yuv420p is lossy; there is no geometric interpolation, scaling,
or invented physical trajectory between samples. Uneven source timestamps are
retained individually, with each state's exact presentation frame range.

`manifest.json` is written after ffprobe confirms codec, dimensions, rate, frame
count and duration, and ffmpeg completes a full decode with error checking.
Duration uses the exact video-track `duration_ts * time_base`, with `start_pts=0`.
The separate MP4 movie-header duration must equal that duration rounded up to the
qualified muxer's 1000 Hz clock; no whole-frame tolerance is permitted. Thus a
182-frame movie at 30 fps has an exact track duration of 182/30 seconds while
the container reports 6.067 seconds. The encoder command and video bytes are
unchanged by this verification rule.
It includes the complete original capture metadata, per-state hold plan, movie
hash, tool hashes and exact commands. `ffprobe.json`, `encode.log`, `ffprobe.log`
and `decode.log` retain verification evidence. A failure leaves partial output
and logs without a successful manifest; retry with a fresh output directory.

Host tests use tiny PNGs and mocked media processes; they never execute ffmpeg:

```sh
python3 -B -m unittest discover -s viewer/video/tests -v
```

## Robodyna-branded presentation copies

`//apps/media:brand_video` accepts a qualified movie directory and creates a new
presentation directory:

```sh
bazel-bin/apps/media/brand_video SOURCE_VIDEO_DIR NEW_OUTPUT_DIR \
  --logo assets/brand/robodyna-logo-primary.png --ffmpeg /path/to/ffmpeg \
  --poster-frame 900
```

The poster index is a zero-based frame in the final movie. Run through the existing
workstation guard. This command overlays the declared top-right logo panel and
re-encodes H.264; it preserves the canvas, frame count, rate and exact track duration.
It verifies source/tool identities, fully decodes the movie and poster, and records
`robodyna.branded_video.v1` separately from numerical qualification. Original movies,
PNGs and simulation archives remain unchanged. It does not advance a simulation.
