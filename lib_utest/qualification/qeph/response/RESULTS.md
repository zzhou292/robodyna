# BQ4 sustained elastic response execution

2026-09-09: the first build, all eleven observer/report test functions, three
existing BQ3 regressions, six actual CUDA response runs and both frozen
refinement comparisons pass. The pre-execution README, design and source map
remain unchanged. No physics, timestep, amplitude or tolerance was adjusted.

Each one/two-cell case completes 4096, 8192 and 16384 accepted intervals at
h/h2/h4 through 244.140625 microseconds. This gives 57,344 owner intervals and
86,016 native/CUDA element intervals, with 257 common endpoint observations
per run. All complete native fields, timing, work and momentum checks precede
each joint owner/history/cache commit.

| Frozen case | Maximum scaled h/h2 difference | Maximum scaled h2/h4 difference | Maximum work-residual ratios at h / h2 / h4 |
|---|---:|---:|---|
| One Q4, balanced rotary pulse | 2.8852668304812103e-6 | 7.213158458659227e-7 | 0.0010805221 / 0.0005403050 / 0.0002701635 |
| Two Q4s, balanced force pulse | 2.133777932334506e-5 | 5.335036049342189e-6 | 0.0013702312 / 0.0006851228 / 0.0003425632 |

Both pass the unchanged 0.02/0.015 response limits, refinement decrease,
nonzero-response/work floors and 0.02 work-residual limit. Differences use
the fixed physical field scales, not relative percentage errors. Residuals
use the common pulse-work normalization and synchronous total-J kinetic energy
plus native EINT0/EINT1/EVIS work; those work fields are not renamed potential
energy. These two refinements do not prove a general temporal order.

The shared-node coarse run reaches displacement/side 7.48394e-6 and rotation
3.05572e-5 rad, within the declared small-response domain. This is a small
elastic qualification, not a crash or vehicle-scale capacity result.

Each run retains the same executable SHA256
`ad7b9b7ef98543d9f41ce19fe10ad83ed1b697c3c072052490289eb8aa991815`
and exact native matrix decision/raw binding. The 88-input source map SHA is
`551217068ee623cbcd1e3566164a6aa4c264d141ccc4a4cf226dce59b66492ba`.
The declared runtime provenance contains 1075 records and reports
`hermetic_runtime: false`. Numeric run hashes match their comparison records.

Evidence lives in workspace `crash-work/runs/qeph-bq4-response-*`,
`crash-work/reports/qeph-bq4-response-*.json`, and corresponding XML files.
The highest sampled run RSS is 164,732,928 bytes; the build peak is
801,570,816 bytes. Each batch keeps one 14,824-byte allocation, plus six
owner allocations totaling 1,780 or 2,602 bytes. Reported device-wide free
memory differences are not allocation measurements.

Twelve distinct new test functions are qualified: eleven host and one runtime
function executed six times. The six runs are not counted as six new functions.
The named free-response gate passes; `simulation_ready` remains false.
Wall contact, T3 coupling, startup impact velocity, plasticity, attachments,
vehicle capacity, 200 ms duration and accepted rendering need further gates.
