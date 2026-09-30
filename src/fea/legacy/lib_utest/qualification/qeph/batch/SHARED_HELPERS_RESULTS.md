# Shared shell utilities regression

2026-09-09. QEPH now uses the same `NodalTrialIdentity` and `ShellBatchFields`
utilities as the resident T3 batch. Exact stamp/view/range comparisons, gathering
and signed cache-work arithmetic moved without changing expressions, reduction
order, mechanics, state storage or the publication boundary. Both utilities are
stateless; formulations retain their own geometry and history. Bazel declares
the direct header dependencies. This is a refactor, not a new numerical gate.

All eight existing QEPH batch functions and three coupled-prefix functions pass.
The six sustained one/two-cell h/h2/h4 runs and both frozen comparison commands
also pass, covering 86,016 native/CUDA element intervals. The complete parsed
scientific records compare exactly against the retained BQ4 baseline: field
dictionaries, common endpoint samples, final fields, work and impulse ledgers,
extrema, and owned allocation counts/bytes. This compares represented JSON
numbers, not binary object padding. Provenance, elapsed time and device-wide
free-memory observations are explicitly excluded. No tolerances were changed.
Direct Bazel CUDA compilation and the 70-record native/port source verifier pass.
No new distinct test functions are added to the retained count.

Evidence uses the `crash-work/reports/qeph-shared-helpers-` prefix; the six run
directories and two comparison files have the same prefix under `crash-work/runs`.
`exact-response-1.json` lists every compared report and the excluded keys.
The 93-input source map has SHA
`4abab52d4ab162096abb459a157fbad14fdebd2e99ee3e5aafa9897c26e7d0bf`.
The current 1,081-record declared runtime provenance has SHA
`2536d63bba42f2b0ff9f56dc63043c5ddda53aac98f585cb625f2bf814939f6e`;
it does not pin a complete system image. The observed runtime binary SHA is
`a894486d1216c3d4e3b0fb42244bb392c600e652411564b19e4e14bfca80ead6`.
The original BQ4 checkpoint and source map remain unchanged. This retains the
same named small elastic free-response scope; contact and mixed-element
recurrence still need their own qualification.
