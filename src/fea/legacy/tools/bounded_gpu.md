# Optional compute-process GPU diagnostics

Pass `--gpu-process-diagnostics` together with `--gpu INDEX` to `run_bounded.py`.
The existing whole-device free-memory and growth guards, CPU/RSS limits,
reason ordering, timeout, cooperative-stop policy and owned-session cleanup
remain authoritative. The option defaults off and does not affect solver state.

The helper queries `nvidia-smi -i INDEX --query-compute-apps=pid,used_gpu_memory
--format=csv,noheader,nounits` at the existing two-second GPU polls. Hard sampled
limits are checked before optional poll diagnostics. On a forced stop caused by
GPU growth, it takes another snapshot before terminating the owned session.
The whole-device poll time and diagnostic start/end times are recorded separately.

Output is capped at 64 KiB while draining the query pipe; stderr is discarded.
The query deadline is one second, with at most one further second for killing
and reaping that direct query child. Process identity snapshots inspect at most
4,097 rows and retain at most 4,096. At most 64 compute rows per snapshot, 128
rolling snapshots, the first snapshot and the last pre-stop snapshot are retained.
Counts, omitted rows, bounded-history metadata and incomplete inventories are
explicit. Optional errors do not change the hard guard outcome or prevent cleanup
and receipt publication. Interruptions remain operative during ordinary sampling.

The existing OwnedSession identity checks authenticate the leader around each
bounded process snapshot. Only stable live PID/start-time/session identities
across the GPU query are classified as owned or other. Missing, clipped, exited
or reused identities are unknown. N/A memory stays null. Totals are known-memory
subtotals of retained compute rows, never complete GPU allocation totals.

Compute-process accounting omits graphics and may differ from driver/context,
MIG/MPS or whole-device accounting; these observations are not atomic. Neither
missing process rows nor a remainder establishes external interference or a leak.
The observations cannot retrospectively attribute the stopped gate-7 memory rise.

Host qualification uses `test_bounded_gpu.py`, the diagnostic regressions in
`test_bounded_history.py`, and existing session/runner tests. They use synthetic
GPU data and mock queries. A separate serialized real-device smoke is needed to
verify that the installed driver actually provides useful per-process memory.
