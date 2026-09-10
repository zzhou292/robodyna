# Authenticated derived-reader qualification results

All five new reader functions and 62 retained wall-analysis/report functions
pass. The first test build found a local namespace alias collision with an
existing fixture namespace. Renaming only that alias fixes the build; all six
reader library modules compiled before that test-only correction. Original
source/map/library and diagnostic are retained under
`crash-work/reports/qeph-wall-derived-reader-failure-1/`.

Tests exercise a full synthetic 87-file report with raw gain 80 retained as a
diagnostic and failed 4H0, plus partial nonfinite evidence, external bindings,
missing/extra/symlink files, byte limits, and rehashed context/operator/spectrum/
Gram/contact/comparison mutations. Rejection preserves the complete caller
result. Compact per-step summaries use reconstructed D/windows/B operators and
original cheap numerical predicates; no native maps or repeated eigensolves.
Synthetic measurements are explicitly labelled and are not physics evidence.

Largest sampled build RSS was 404,602,880 B.
The corrected build took 4.147 s; tests took
3.637 s under one CPU/1 GiB, with no GPU. Resource guards passed
on both build attempts; the first command exited on the compiler error.
The corrected [223-input map](derived-reader-source-map-r2.json) has SHA256
`30c7b49b5e6e1a8d610b5d0ea24776aa2b463968ce1217459d5b02c1d7819e28`;
the original map remains unchanged. Reports and XML use
`crash-work/reports/qeph-wall-derived-reader-*`. The
`qeph-wall-derived-reader-1` checkpoint preserves actual source/runtime.

Actual six-job derived analysis already passes; executing this reader and the
four boost comparisons in the final selection command is next. No selected
impact timestep or incoming CUDA trajectory is claimed by the reader unit gate.
