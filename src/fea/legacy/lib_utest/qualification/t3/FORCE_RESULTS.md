# T3 force/history port execution

2026-09-09: the first build and numerical execution pass. The pre-execution
contract remains in `FORCE.md`; its frozen manifest SHA256 is
`b6f59f4849940286841bb985e80bcd187ce3b345b2a30d40c8850905e0b4e312`.

Nine new host and three actual CUDA test functions pass, with 870 native
force/history configurations and 64 independent signed physical modes per
backend. Thirteen malformed/late force cases, all 26 nonfinite history slots,
aliasing, repeated evaluation and clean retry pass. The independent virtual
power/material/work oracles retain their original dimensional tolerances.
All 35 existing startup/rates/native-force regressions also pass: 41 host and
six CUDA function executions in total. The owning Bazel force dependency graph
passes; CMake compiles and executes the actual host/device headers.

The CUDA fixture uses one 2,304-byte allocation and one thread per block.
The compiled sm_120 kernel reports 204 registers, eight stack bytes, zero local
and shared bytes. These are fixture resource measurements, not batch throughput.
Both build and numerical runs use the workspace resource guard.

Evidence: workspace `crash-work/reports/t3-force-port-*-1.json`,
`t3-force-port-{host,cuda}-xml-1/` and
`t3-force-port-cuda-resources-1.txt`. Source verification passes 61 force-stage
records plus 54 startup/rates records and the unchanged native closure.
The live startup manifest records only the reviewed optional build registration
revision; historical snapshots, numerical sources and their test cases remain
unchanged. The new manifest is `9ff2ae68a763b7483ca43d0bf133a75c8956dadc8867155d4211dd5475bcaebe`.

This qualifies the selected prescribed T3 value operation. It does not add
resident T3 history, mixed shared-node dynamics, a timestep admission, original
MAT024/NIP3 behavior, contact or a vehicle trajectory. Production uses the
shared LAW1 helper and CUDA-portable headers; Fortran is a test oracle only.
