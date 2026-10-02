# Retained Vehicle examples

`//examples/vehicle:native_demos` groups 38 real original C++ programs admitted by
the native Vehicle/FEA/rendering and selected sparse-solver profiles. Each `robodyna_cpp_demo` target
compiles the original entry point directly. The articulated-vehicle case also
uses its eleven original local model sources through a dedicated helper library.
Building these targets does not execute them.

[`DemoCatalog.json`](DemoCatalog.json) preserves the complete **64-program**
denominator. Every source now has an actual declared target. The first batch
covers the ordinary wheeled/tracked
models, controllers, checkpoints, rigid/SCM terrains and supported test rigs.

The other 26 have explicit optional-profile targets:

- Eight MPI/co-simulation cases, with robot/SPH/Multicore requirements where used.
- Five FMI cases, including the matching FMU artifacts needed for execution.
- Five SPH/CRM cases.
- Five Multicore cases.
- Three OpenCRG cases.

The existing postprocess library being available does not automatically qualify
that final program's plotting environment. All 64 programs passed the current
full compile matrix; loading their assets and completing physics require
separate runtime evidence. Initial catalog status fields remain an initial
snapshot; current compilation evidence lives in the matrix's per-target receipts.

The module preserves its original source configuration, numerical settings and
physical time steps. No missing optional implementation is replaced with a stub,
an empty preprocessor branch or a different solver to satisfy a build count.
The renderer adapters remain separate reusable targets under
`//src/vehicle/visualization`; core state and SCM ownership are shared.

Use the existing workstation guard for builds and case-specific guards for later
runs. Runtime defaults may be long or interactive. The
[asset and launch plan](RUNTIME_ASSETS.md) explains why compiling a native program
is separate from packaging a closed, portable simulation input set.
