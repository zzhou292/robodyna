# Selection command/report utility qualification

Four new report functions and all 67 retained host functions pass on first
execution. All 76 command protocol cases pass: fourteen retained raw, 31
retained analysis and 31 new selection cases. Malformed preflight requests
create no output. Two late input-binding failures retain only initial provenance
and progress, without a final index. The shared six-field raw parser preserves
the original command semantics and strict binary64 velocity spelling.

The report checks captured child implications, full 194-entry baseline and
matrix/gain links, failed diagnostic 4H0, a complete scientific rejection,
partial/unavailable summaries, exact tuple/byte bindings and create-only failure.
One raw/derived job is loaded at a time, with one immutable zero cache per
fixture. The final report is capped at 2 MiB within the shared 96 MiB allowance.
No native/Schur/Gram calculations are repeated by the selection command.

The guarded one-worker build took 15.264 s, largest sampled
process-group RSS 406,188,032 B. Tests took 4.141 s
under one CPU/1 GiB; no GPU was used. Every resource guard passed. Reports/XML
are `crash-work/reports/qeph-wall-selection-command-*`; the exact CLI assertions
are retained in the command guard report.

The [234-input map](selection-command-source-map.json) has SHA256
`7daf423132ccf7191ae739e0dbb4cb351ee429849581527f89a68742fa906725`.
The `qeph-wall-selection-command-1` checkpoint preserves actual source/runtime.
The six derived jobs already pass; actual authenticated reading, cross-boost
comparison and final selection are the next execution, not claimed by these
synthetic utility tests. [Command contract](SELECTION_COMMAND.md).
