# Active restructuring plan

The user authorized implementation on 2026-09-30. Work on local branch
`work/initial-integration`; destination is https://github.com/zzhou292/robodyna.git.
No publication is part of the current staged operation.

## Checkpoints

1. SOURCE IMPORTS VERIFIED — all three imported trees match pinned Git trees and
   original histories are reachable. App/renderer/postprocess histories consolidated;
   original dirty-doc and dependency differences preserved.129 LFS payloads verified
   and restored locally; all six dependency gitlinks inventoried. Optional dependency
   checkout/build qualification remains explicit follow-up.
2. IN PROGRESS — one root Bazel entry with reusable TL targets, Chrono transition build,
   explicit dependencies, host checks and source-verification tools.
3. IN PROGRESS — real application CLI around existing preparation/run/output libraries;
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

## Evidence so far

- Exact tree imports: source manifest and bounded import receipts in the outer workspace.
- Imported application postprocessing: 32 host tests passed.
- Root Bazel source-boundary checks: seven tests and critical source/patch identity passed.
- Bazel graph resolved; Chrono bridge and native482-TU aggregate compiled.
- Core, rigid, FE, source-inventory and OpenSSL SDK host gates passed.
- Product Python admission and C++ request tests passed.
- Importer preservation/rejection tests passed (six cases).
- Full native vehicle backend build is in progress; GPU parity and independent
  FEA/MBD gates remain pending.
