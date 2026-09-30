# Native raw capture runner

`qeph_wall_raw CELLS BOOST PROVENANCE_FILE EXPECTED_PROVENANCE_SHA256 NEW_DIRECTORY REMAINING_SET_BYTES`
accepts canonical cells 1/2, boost -8/0/8 and positive decimal byte allowance
at most 96 MiB. Root serializes jobs and reserves derived-report capacity; the
allowance is never an independent 96 MiB budget for each job.

Before directory creation, the thin CLI checks exact provenance bytes/hash,
its mandatory `executable:{path,sha256}` binding against the running image,
input/output paths and all argument forms. Source-map authentication remains
the root launcher's responsibility; self-supplied hashes do not establish it.
The existing writer stores exact provenance unchanged and persists each capture
callback. Exit 0 means full raw collection, 2 incomplete numerical collection,
and 1 protocol/IO failure. Earlier artifacts survive exceptions. Local check
failures remain data; none of these return codes admits a simulation.

The shared ContactDerivativeChecks header extracts the already qualified
one-sided quotient and directed comparison arithmetic verbatim. It supports
both collection and subsequent checks of retained data without duplicating
budgets. The old native model/probe and raw/analysis/support regression bodies
remain unchanged. Root must run them and CLI preflight checks before full jobs.
