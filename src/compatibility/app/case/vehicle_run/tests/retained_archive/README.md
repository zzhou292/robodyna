# Retained two-interval archive qualification

The live two-interval gate and this host-only target share one post-run checker.
The live gate still checks its actual source forecast and controller result before
calling it. The retained gate requires explicit viewer-input and summary SHA-256
pins, verifies diagnostic interval-limit status, and reads the same closed archive.
It creates no dynamics owner, retries no step, edits no artifact and claims no
physical restart. Both use the existing authenticated Replay and interval reader.

The original extra interval read used a fixed 16 MiB cap. The selected self-contact
profile needs 29,401,176 bytes under the existing conservative rule: 25001 planned
rows, 49 typed columns and three staging copies. IntervalReadStagingBytes exposes
that exact rule; admission, chunking, format and the 256 MiB hard reader ceiling are
unchanged. The checker supplies the calculated cap, not an arbitrary large limit.
Replay's existing 512 MiB source/frame budget remains unchanged. A small host test
covers the long-horizon/two-row prefix, exact cap, one-byte-short rejection before
callbacks, empty-prefix behavior and retry.

Configure this directory as a CXX-only project with explicit Chrono_DIR and
ROBO_DYNA_TL_ROOT. It links archive/Chrono host code and GTest, no live solver or
CUDA language. Run only in the root's free lane, with two affinity CPUs, 10 GiB
sampled RSS and 32 GiB available RAM. Required environment:

- ROBO_RETAINED_V5_RUN: existing completed controller output directory.
- ROBO_RETAINED_V5_VIEWER_SHA256: caller-pinned viewer-input.json hash.
- ROBO_RETAINED_V5_SUMMARY_SHA256: caller-pinned run-summary.json hash.

GTest: VehicleRunRetainedArchive.TwoCommittedV5IntervalsMatchPinnedSummaryAndSource.
CTest: vehicle_run_retained_two_intervals.

A pass validates retained record readback only. Preserve the original gate 14 exit 1
and its exact readback-cap exception. Recovery must separately authenticate that
this failure occurred after two real commits/closed output, then require this test
and actual native Chrono replay. Do not run the old PASS-only postrun proposal or
pretend the original failed GTest passed. No new useful 400 ns crash movie is implied.
