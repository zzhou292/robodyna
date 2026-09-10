# Boost comparison and timestep-selection utility results

All seven new host functions and 55 retained wall-analysis/report functions
passed on first execution. Tests cover exact +/- velocity lift, unchanged full
state matrices, physical-unit/model/window errors, partial-size rejection before
cache publication, late failure/retry, missing/duplicate tuple grids and the
frozen H0/H0/2 margin. A failed diagnostic 4H0 does not veto an otherwise valid
selection. Raw gain remains diagnostic; numerical completion still requires
finite raw and weighted measurements.

The guarded build took 5.948 s with largest sampled process-group
RSS 374,087,680 B. The test command took 2.599 s
within one CPU/1 GiB; no GPU, eigensolves or native intervals were added by these
seven functions. All guard checks passed. Reports are in parent-workspace
`crash-work/reports/qeph-wall-selection-*-1.json` and `qeph-wall-selection-xml-1/`.

The [input map](selection-source-map.json) pins 215 canonical source paths, SHA
`64adbb83d1ebd08f58ff488b81762f32e384a4dabbc4b5ab44b686e0c487c4aa`.
The `qeph-wall-selection-1` checkpoint retains the qualified source/runtime.
Actual six-job analyses already pass, but authenticated derived reading and
final cross-boost command execution remain pending. This utility result alone
does not select a timestep or admit an incoming CUDA trajectory.
