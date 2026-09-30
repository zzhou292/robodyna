# Actual full-state derived screen results

All six authenticated raw jobs completed analysis on first execution. Every
sampled step/amplitude and physical branch recheck passes, including diagnostic
4H0. The 108 amplitude analyses retain all 109/194 coordinates, raw/weighted
constant-branch spectra, eleven chronological Gram analyses, moving baselines
and fresh directional quotient checks. This is a local recurrence screen;
cross-boost comparison and the final factor-two selector remain pending.

The maximum measured constant-branch spectral radius is 1.0000000047784086,
below 1+5e-8. Maximum weighted mean gain is 45.546286387450365, below 64.
All 522 derived files and externally pinned indices verify; derived bytes total
42,786,429. Including 47,302,880 raw bytes leaves 10,573,987 in the shared 96 MiB
budget. Selection output has a reserved 2 MiB allowance. Dense B/D operators
are exactly reconstructible from their raw matrices and retained descriptors.

Each serial analysis used one CPU, a 1 GiB process-group RSS limit and a
120-second deadline; no GPU or new native intervals were used. One-cell jobs
took 5.948–5.952 s; two-cell jobs took 29.972–30.235 s. Largest sampled process-
group RSS was 54,325,248 B. Every guard passed.

The executable SHA256 was
`cfaaef678ecbe26e89906e3dbc789689c6bb2a4c27a797583065c335015b39b7`.
Exact per-job configurations bind the reviewed 220-input source map, 57 declared
linked inputs, raw index/provenance and exact raw byte count. Before each run,
all declared source/build bytes were rechecked. Compiler/native provenance has
the existing delegated nonhermetic scope. The source/runtime dependency is
`qeph-wall-derived-command-1` at TL `23f05a1`; raw input is the immutable
`qeph-wall-native-raw-1` checkpoint at `bd21204`.

See parent-workspace `crash-work/reports/qeph-wall-derived-inventory-1.json`,
`qeph-wall-derived-launch-plan-1.json`, six guarded reports and six config files.
The `qeph-wall-native-derived-1` checkpoint retains the exact runtime/source
snapshot and all analysis artifacts. No incoming trajectory or vehicle is admitted.
