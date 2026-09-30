# Derived report and command qualification results

Four new report functions and all 51 retained host functions passed on first
execution. Fourteen existing raw command preflight cases and 31 analysis command
rejection cases also passed, with no output directory created on rejection.
The writer preserves full spectral/Gram evidence, raw links, exact B/D operator
hashes, ordered progress and partial failures. Captured child statuses cannot
be upgraded into a completed step without corresponding evidence.

The guarded build took 9.619 seconds; largest sampled process-group
RSS was 427,573,248 B. Tests took 2.077 seconds
under one CPU/1 GiB; no GPU or new native intervals were used. Every guard passed.
Reports are `crash-work/reports/qeph-wall-derived-command-*-1.json`, with XML in
`qeph-wall-derived-command-xml-1/`. CLI assertions are retained in the guard report.

The [input map](derived-command-source-map.json) pins 220 reviewed inputs:
SHA256 `9e9de5da418aabe8094f0e31f8c4756be5e8e8d6f82120153b5f8fba545b0d6c`.
The `qeph-wall-derived-command-1` checkpoint preserves source and runtime evidence.
These utility tests do not select an impact timestep. Actual six-job analysis,
cross-boost consistency and selection remain the next numerical gate.
