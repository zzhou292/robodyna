# Optional accepted-prefix stop request

The default `run_bounded.py` behavior is unchanged: a sampled GPU-growth breach
stops only the launched process group. Growth measures the selected device's
total usage relative to launch, so another job can cause that breach. The guard
does not attribute allocations or control unrelated jobs.

For a controller that already honors an existence-based stop file, opt in with
`--cooperative-stop-file PATH` and optionally
`--cooperative-stop-grace-seconds 30`. The grace defaults to 30 seconds and must
be finite and positive. A selected `--gpu` is required. Pass the same absolute
path to the vehicle CLI's existing `--stop-file PATH`; the guard does not edit
the child command or assume it implements this protocol.

The path's parent must exist and the path must be absent, including symlinks.
Use a fresh sibling of the run output directory, not a path inside a directory
the controller requires to be initially empty. Report/lock aliases reject.
The request uses exclusive creation and never truncates an existing file.
It remains after the run for diagnosis; select a new path for the next run.

Only GPU growth can initiate grace, and only after that sample's available
host RAM, owned-group RSS and free-GPU checks pass. Command timeout stays hard.
During grace the existing monitoring continues; hard-limit breach, timeout,
request failure or grace expiry uses the existing SIGTERM/SIGKILL owned-group
cleanup. A later fall in GPU usage does not cancel an issued stop request.
These remain sampled limits, not a cgroup or guarantee between samples. GPU
polling remains every two seconds and host polling every quarter second.

For a successful controller exit after requesting stop, the guard returns the
child's zero exit code with report status `cooperatively_stopped`, rather than
`passed`. This proves only the observed request/exit; the controller's own
manifest must authenticate an accepted prefix and its completed physical time.
Nonzero exits remain `command_failed` with their exit code. Forced termination
remains `blocked_or_stopped`, exit 125.

The optional `cooperative_stop` report contains the triggering sample/reason,
absolute file path, request time and creation/completion flags, grace deadline,
and cooperative exit or forced-stop result. Forced stops record termination
request/completion times; request I/O failure preserves its distinct outcome.
No such report key is added when the option is absent.

Example wiring (substitute the already qualified command and normal limits):

```sh
python3 -B Total-Lagrangian-FEA/tools/run_bounded.py \
  --report crash-work/reports/next-preview-guard.json \
  --lock crash-work/reports/workstation.lock \
  --gpu 0 --min-gpu-free-gib 8 --max-gpu-growth-gib 6 \
  --max-rss-gib 20 --min-available-gib 32 --timeout 7200 \
  --cooperative-stop-file /absolute/fresh/next-preview.stop \
  --cooperative-stop-grace-seconds 30 -- \
  /absolute/qualified/vehicle-run [existing run arguments] \
  --stop-file /absolute/fresh/next-preview.stop
```

Run the owning host tests from the TL checkout:

```sh
python3 -B -m unittest discover -s tools -p test_run_bounded.py -v
python3 -B -m unittest discover -s tools -p test_cooperative_stop.py -v
```

The five existing tests remain unchanged. Twelve new functions use real tiny
Python children and mocked RAM/RSS/GPU metrics; they never query a GPU. They
cover default kill, successful prefix/zero exit, nonzero exit, grace expiry
after growth recovery, all hard limits during grace, timeout, no-growth normal
exit, initial hard rejection, path collision/symlink/alias protection and invalid
options. Actual vehicle accepted-prefix finalization is a separate root gate.
Historical source receipts are retained unchanged.
