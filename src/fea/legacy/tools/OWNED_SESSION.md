# Owned-session workstation supervision

`run_bounded.py` now charges every readable process in the session created for
its direct child, including descendants that create separate process groups.
CLI options, CPU affinity, limits, sampling cadence, retained history, resource
failure ordering and optional cooperative-stop grace are unchanged.

This repairs an observed Bazel accounting omission. In the root-owned
`cin-group-owning-build-1` compilation, Java/outer sandbox used PGID 3215353;
the inner sandbox used PGID 3218868, and NVCC/ptxas used PGID 3218869. All had
SID 3215353. One host metadata snapshot showed 447,070,208 bytes in the selected
PGID and another 1,611,198,464 bytes in its other groups. The final old report
recorded only 623,620,096 bytes peak sampled RSS. This proves undercounting,
not a budget breach or an observed surviving compiler after termination.

The original report and historical source receipts remain unchanged. The old
guard is preserved at commit `3901024`, SHA-256
`811bbd54ea171f29c33932858957b937dc651baadb57dff2c2d1dafaabd9ec65`.
No historical report is reinterpreted as full-session telemetry.

## Identity, sampling and termination

The runner checks Linux `pidfd_open`, `pidfd_send_signal` and
`waitid(WNOWAIT)` support before launching. SIGCHLD must use its default
disposition so the leader's exit status remains available.

After `Popen(start_new_session=True)`, the helper authenticates the direct
child's PID, parent PID, process group, session and `/proc` start ticks. The
leader remains unreaped throughout sampling and cleanup. `waitid` observes its
exit without consuming status; its numeric PID/session therefore cannot be
reused while the helper still relies on them. Session scans check the anchor
before and after enumerating rows. RSS, live CPU ticks and thread totals include
all observed PGIDs in that SID. No process names, command lines or environments
are used for selection.

Cleanup sends TERM to each live observed member through a pidfd, checking its
PID/start ticks/session again after opening that handle. A numeric PID reuse
cannot redirect the signal to a different process. Every 50 ms, cleanup scans
again for surviving or newly observed members, sending TERM once per currently
tracked identity. After the existing two-second TERM grace, it rescans and
sends KILL, with a further bounded two-second drain. No group-wide signal or
unrelated-session process is selected. Only current row identities and one
temporary pidfd are retained, not a whole-run process inventory.

Zombies and dead rows need no signal. The direct child is reaped only after
session cleanup completes, preserving its actual zero/nonzero exit status.
If live members remain or identity is lost, the run reports cleanup failure
rather than success. An existing resource/grace failure reason stays primary;
`cleanup_error` adds the cleanup detail. A cooperative failure then records
`termination_failed_elapsed_seconds` instead of claiming termination completed.
The workstation lock stays held through exceptional cleanup and final report
writing, so a second guard cannot launch during the TERM/KILL grace.

The additive `process_scope` receipt identifies `owned_session_v1`, leader
PID/start ticks/SID and cleanup outcome. A session-authentication failure claims
only direct-child cleanup and remains a failed run.

## Deliberate boundaries

This is a sampled Linux development guard, not a cgroup or security sandbox.
A descendant that deliberately calls `setsid()` leaves the tracked session;
external daemon/server work is also outside this profile. No containment of
those cases is claimed. Processes may change or exit between `/proc` reads;
unreadable unrelated entries are skipped. Sampling does not establish peaks
between polls, and summing RSS can count shared pages more than once. Existing
whole-host RAM reserve and device-wide GPU limits still apply independently.

## Qualification

From this checkout, run the complete small host suite through the normal guard:

```sh
python3 -B tools/run_bounded.py \
  --report /absolute/fresh/bounded-session-root.json \
  --lock /absolute/author.lock --cpus 1 \
  --min-available-gib 1 --max-rss-gib .5 --timeout 100 -- \
  python3 -B -m unittest discover -s tools -p 'test_*.py' -v
```

The 44 tests include all 26 preceding guard/history/cooperative tests, fake
process-table identity/reuse/disappearing-row controls, cleanup failure reason
preservation, and real tiny children. Real controls cover a 64 MiB allocation
in another PGID, timeout, runner TERM, ignored TERM followed by KILL, parent
exit 2 with a surviving child, fast exit 0/2, and an unrelated session that
remains running. A contender guard must also reject during cleanup grace while
the workstation lock is still held. These tests never query a GPU. The next actual owning Bazel
compilation is a separate root qualification of session-wide telemetry.
