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
viewer.postprocess.test_replay_evidence viewer.postprocess.test_native_run -v` (one command).

Closed-output dispatch supports the existing vehicle `run-summary.json` contract
unchanged, and the explicit native `summary.json` schema
`robo_dyna.native_shell_impact_run.v1`. The native closure precheck validates its nested
RecordFiles, binds the forecast to the authenticated native configuration/profile,
and checks exact final archive count/time and complete/prefix claims before
returning an in-memory postprocessing view. It never writes a vehicle summary
or converts a failed producer guard into success. The existing C++ exact archive
checker remains responsible for full schema, identity and physical-frame validation. Notifications name the actual
run directory/profile; the renderer labels physical time in microseconds and
milliseconds without changing stored samples or geometry scale.

The separate `robo_dyna.native_vehicle_contact_run.v1` summary now dispatches to
`native_vehicle_run.py`. It binds the complete physical native-group/environment
profile, exact configured horizon, final sample and typed declared interface
order through the same read-only record hashes. It only normalizes an in-memory
operator view; no small-coupon or legacy summary is written. For the owning
full-case target, set both launch `output` and postprocess `run` to the genuine
`.../accepted` child, which contains `viewer-input.json`, `summary.json` and
`archive/`. The outer qualification `case.json` is a separate report.

The existing C++ viewer already accepts this `accepted/` directory or its exact
`viewer-input.json`, with `--receipt-sha256` binding the immutable receipt. It
opens the same generic physical-run reader and validates every native-group
record and frame before capture. No new renderer or archive rewrite is needed.
A guarded GoogleTest exit 1 remains a failed delivery qualification even when a
truthful diagnostic prefix closed; automatic postprocessing continues to reject
that exit. A separately authorized diagnostic replay can use the direct C++
receipt route without changing that failure report. Normal completed exit 0 and
typed prefix exit 2 retain the existing automated policy.

For 61 recorded frames the existing 2GiB PNG reservation admits the capture
(`61 * 32MiB + 4MiB`). Use an existing full-car view and a separate fixed-camera
front close-up at scale 1; camera arguments select views without altering any
recorded coordinates. Run the existing exact archive checker before capture and
fully decode the final encoded MP4 as the normal postprocess job already does.

Additional host test: `python3 -B -m unittest
viewer.postprocess.test_native_vehicle_run -v`. These tests validate closure and
hash behavior only; complete C++ record/geometry checks remain mandatory.
