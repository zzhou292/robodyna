# Screened broadside rebound: retained result

All six one/two-cell runs and both frozen h/h2/h4 comparisons pass at source
commit `b5811eb`. Each trajectory reaches 244.140625 microseconds with 257
common output samples; together they execute 57,344 owner intervals and 86,016
native/CUDA cell intervals. The finite triangle-mesh wall remains the actual
contact geometry. This validates the named elastic broadside fixture, with
rigid translation and zero deformation; it does not qualify deforming impact,
wall tessellation variants, mixed response or a Yaris trajectory.

| Cells | Step | Accepted intervals | Maximum penetration mm | Final upper normal velocity m/s | Runtime s |
|---|---|---:|---:|---:|---:|
| one | h | 4,096 | 0.375000002 | -7.9999996926 | 2.972 |
| one | h2 | 8,192 | 0.375000005 | -8.0000001299 | 5.857 |
| one | h4 | 16,384 | 0.374999999 | -7.9999999117 | 11.590 |
| two | h | 4,096 | 0.375000002 | -7.9999996926 | 4.226 |
| two | h2 | 8,192 | 0.375000005 | -8.0000001299 | 8.367 |
| two | h4 | 16,384 | 0.374999999 | -7.9999999117 | 16.678 |

The selected step is 2^-24 s and penalty is 5917682346.6666698 N/m^3, unchanged
from the accepted full-state screen. Entry, peak compression and exit are
bracketed at every accepted endpoint. The coarse run enters between epochs
196/197, reverses between 1431/1432 and exits between 2667/2668. Outgoing
velocity approaches -8 m/s, against the independent per-node spring solution.
All deformation, rotation, strain and curvature measures remain exactly zero;
reference area and thickness ratios remain one.

For both fixtures, coarse/medium and medium/fine maximum normalized field
upper differences are 1.5852799562e-7 and 8.9264497703e-8. Maximum synchronous
energy-residual upper ratios at h/h2/h4 are 3.854674e-7, 6.850562e-8 and
2.482318e-8. Analytic uncertainty upper differences are below 0.001186 at h
and decrease under refinement. The frozen response, contraction, energy,
analytic, coverage and cap checks all pass without changed tolerances.

The 17 new observer/comparison/report/transaction functions pass, including
nonzero internal-force synchronization and failed-transaction/exact-retry
coverage. Eighteen affected existing regressions also pass. Initial build and
observer failures and their unchanged test operands are described in README.
The full runner adds one distinct function executed under six configurations.

Evidence: `crash-work/runs/qeph-wall-response-{one,two}-{h,h2,h4}-1/`,
`crash-work/runs/qeph-wall-response-{one,two}-comparison-1.json`, and matching
`crash-work/reports/qeph-wall-response-*` logs, guard reports and XML. Exact
shared launch provenance is `qeph-wall-response-provenance-1.json` (2,393 B),
SHA256 `5ac5d7062ae78f0b8d590f4d40fcd6629dfec71af14af938219c823b53f99eaf`.
The run executable SHA256 is
`0153f04471cf55ce8768bdedfee17239c2854500d0cccefe8701643ae3f5eb91`.
Comparison records retain each exact input hash. Reviewed source commits,
existing native-source provenance and build/test reports replace an additional
large per-utility snapshot.

All six guarded runs take 51.906 s total and peak at 172,756,992 B sampled RSS;
each uses one CPU, a 1 GiB RSS ceiling, monitored GPU0 and the workstation
lock. Both host comparisons pass in under one second combined. Device storage
remains eight allocations, 116,028/116,850 B for one/two cells. These observed
fixture costs are not a vehicle throughput estimate.

The next delivery work is joined mixed Q4/T3 feedback, accepted phase-aware
output and connected capacity for the complete 117-node source part. Additional
broadside variants are deferred unless an integration failure requires them.
