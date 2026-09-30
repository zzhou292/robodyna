# Active restructuring plan

The user authorized implementation on 2026-09-30. Work on local branch
`work/initial-integration`; destination is https://github.com/zzhou292/robodyna.git.
No publication is part of the current staged operation.

## Checkpoints

1. IN PROGRESS — preserve source and dirty-state evidence, consolidate qualified
   app/renderer/postprocess histories, import exact TL/Chrono source trees.
2. PENDING — one root Bazel entry with reusable TL targets, Chrono transition build,
   explicit dependencies, host checks and source-verification tools.
3. PENDING — real application CLI around existing preparation/run/output libraries;
   exact short Yaris regression and replay/video compatibility.
4. PENDING — extract neutral mechanics seams, prove FEA-only/MBD-only link closures
   and a coupled flexible-beam/rigid-mechanism test.
5. PENDING — migrate remaining Chrono modules and supporting APIs against a complete
   capability matrix, then qualify CUDA coverage by module.

Review the plan after each checkpoint or unexpected dependency/physics difference.
A source import is not a successful build; an aggregate compatibility target is not
FEA/MBD independence. Preserve every old source tree, checkpoint and failure receipt.

## Resource policy

Build: eight affinity CPUs, four compiler workers,16 GiB sampled RSS and32 GiB
available RAM. Focused GPU tests: two affinity CPUs,10 GiB RSS, GPU0 free reserve
8 GiB, whole-device growth6 GiB unless a separately authorized scoped recipe applies.
Full vehicle: four CPUs,18 GiB RSS,8 GiB GPU free,9 GiB growth under its approved
profile. Serialize heavy jobs with crash-work/reports/workstation.lock in the outer
workspace. Manage Bazel server lifetime; use batch mode for bounded qualification.

Do not rerun the11-hour100ms trajectory for packaging-only edits. Begin with host,
source and short native coupons; run longer physics only when a change warrants it.
