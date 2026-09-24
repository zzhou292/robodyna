# Render after a local simulation

`python3 -B -m viewer.postprocess CONFIG.json` waits for the specifically
identified simulation guard to exit, then sequences the existing exact Chrono
archive checker, physical replay capture, and `viewer.video.encode`. Run from
the application directory, or put that directory on `PYTHONPATH`.

The small waiting process does not acquire the workstation lock, signal the
simulation, read a changing archive, restart mechanics, or use the GPU. Each
postprocessing stage acquires the existing `run_bounded.py` workstation lock.
Stages use two affinity CPUs, a 10 GiB sampled RSS ceiling, and a 32 GiB available
RAM reserve. Capture additionally enforces the existing GPU free/growth limits.

A configuration specifies absolute paths for `workspace`, `application`, `run`,
`launch_receipt`, `job_directory`, `guard`, `workstation_lock`, `scene_checker`,
`viewer`, and `ffmpeg`; `wait_timeout_s`, `requested_steps`, environment values,
tool-path-to-SHA256 `pinned_files`, and `views` containing a name and viewer camera
arguments. Job directories are single-use. Preserve a failed directory and use
a new one for a reviewed retry.

Only a closed, hash-consistent accepted archive is renderable. Exit code 2 is
valid for an intentional diagnostic prefix. A shorter closed run is labeled as
stopped early, rather than reported as the requested step count. The exact
all-node/part-color/activity replay test must execute and pass before capture.
`replay_evidence.py` verifies the named completed GoogleTest case and consistent
suite/report counts; empty aggregate reports, skipped cases and other test
identities cannot authorize rendering. The same test identifier selects the
checker invocation and validates its report.
No recovered samples are inferred automatically from missing final output.

Each view keeps original part colors, physical deformation scale 1, the finite
mesh wall and saved timestamps. At five saved states per presentation second,
41 states produce an 8.2-second video; this is slow playback of the actual
physical interval. The encoder publishes metadata only after complete decoding.
Automatic checks do not certify physical validity or camera composition.

`status.json` records the current stage, accepted duration/count, video paths,
and notification results. GNOME desktop notifications announce simulation
completion and video readiness when the configured user session is available.
They are local desktop notifications, not scheduled chat messages. Notification
failure is recorded separately from simulation or video validity. Source
archives are never modified.

Host tests: `python3 -B -m unittest viewer.postprocess.test_lifecycle
viewer.postprocess.test_replay_evidence -v` (one command).
