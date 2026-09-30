# Native recurrence audit execution

2026-09-09: six audit and three report test functions pass on their first
execution. The full native audit also passes all 36 fixture/step/amplitude
combinations. The frozen contract and source map remain unchanged.

The complete recurrence dimensions are 109 and 194, including geometry,
velocities, material history and cached internal forces. Feedback dimensions
are 68 and 124; excluded feedback measures exactly zero before reduction.
Maximum finite-difference matrix discrepancy is 1.8189894035458565e-10.
The maximum feedback spectral radius is 1.000000000000004 and the maximum
finite-horizon Gram mean gain is 25.612799748208108, within the declared
numerical budgets. This gain is a mean-square measure, not a uniform bound.

The declared selection rule chooses H0 = 2^-24 seconds. All larger tested
steps passing does not authorize changing the declared response grid.
Maximum Schur residual is 3.868269361664756e-15; the report retains all full
matrices, diagnostics and fixed input dictionaries.

Evidence lives in workspace `crash-work/reports/qeph-bq4-audit-*-1.json`,
`qeph-bq4-native-audit-1.json`, the host XML directory and
`crash-work/runs/qeph-bq4-native-audit-1/`. The raw matrix report is
5,048,285 bytes with SHA256
`4e0a94d32652a81d347ef2c7317f3688fe505e0e11dedc5e916512d08af74313`.
The declared runtime provenance records 719 files and explicitly reports
`hermetic_runtime: false`. Independent source/result review found no blocker.

This is a native local recurrence screen. Both simulation readiness and
trajectory qualification remain false. Sustained CUDA response, independent
work balance and the frozen h/h2/h4 comparisons must pass separately.
