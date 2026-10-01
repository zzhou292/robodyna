# Active restructuring plan

The user authorized implementation on 2026-09-30. Work on local branch
`work/initial-integration`; destination is https://github.com/zzhou292/robodyna.git.
The user subsequently authorized initial publication to this repository after
retained Chrono demonstrations and a Robodyna logo are complete. Those tasks
have passed their runtime/media gates; final publication is being prepared.

## Checkpoints

1. SOURCE IMPORTS VERIFIED — all three imported trees match pinned Git trees and
   original histories are reachable. App/renderer/postprocess histories consolidated;
   original dirty-doc and dependency differences preserved.129 LFS payloads verified
   and restored locally; all six dependency gitlinks inventoried. Optional dependency
   checkout/build qualification remains explicit follow-up.
2. BUILD CHECKPOINT PASSED — one root Bazel entry builds the retained TL CUDA
   backend, native Chrono core/FEA aggregate, production application and native
   VSG viewer. Host, source and SDK checks passed. These aggregate targets are
   transitional; they do not establish independent FEA and MBD modules.
3. PHYSICS CHECKPOINT PASSED — normal application CLI completed 101 Yaris steps;
   all 50 archived files and viewer receipt match the frozen baseline byte for byte.
   Every non-timing summary field matches. Both stored frames passed C++ replay.
   Native viewer execution and rendering passed on the short run and the preserved
   100 ms archive. The new 60.2-second MP4 is byte-identical to the original
   qualified video, fully decoded and visually reviewed; inputs remain unchanged.
4. IN PROGRESS — 21 unchanged source files now compile in neutral foundation,
   mass-block, frame and inertia libraries. Actual header/link and 482-source
   ownership gates and all 20 runtime/build test targets passed;
   FEA-only/MBD-only closures and a coupled flexible-beam/body gate remain next.
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
- Full native vehicle backend build passed (`robodyna-native-backend-build-3`).
- Native VSG viewer and three SDK/asset/value test targets passed
  (`robodyna-viewer-native-tests-2`), without linking an old Chrono library.
- CLI launch guard now capability-checks its Python interpreter. A real monitored
  `/bin/true` passed with complete cleanup; 27 driver behavioral host tests passed.
- Product package attempt3 passed. The actual 101-step GPU run passed in
  22.569 seconds including setup and replay; the independent comparison passed.
  Receipt paths, hashes and exact comparison scope are in `QUALIFICATION.json`.
- The neutral extraction passed 20 test targets; its new 101-step run again
  matched all 50 archive files, viewer receipt and non-timing fields exactly.
- The actual native product render/encode/decode workflow passed on that run,
  and the preserved 100 ms delivery now renders identically through the product.
- Independent FEA/MBD gates remain pending; `NEXT_SEAMS.md` records the next
  source-audited visual, system-service, contact and assembly extractions.

## Next architectural extraction

The first small dependency split will own the existing moving-frame and inertia
implementations under neutral mechanics, with only core/archive and variable-block
support. Standalone wrench and inertia tests must link without the aggregate.
This changes ownership, not equations. Preserve inherited include and archive names.

Source review also found a visualization dependency through `ChObj` and
`ChVisualShapeFEA`. Moving the entire participant hierarchy without resolving that
dependency would hide FEA inside a supposedly neutral package. Treat visual adapters,
mesh/system services, contact reporting and mixed assembly as explicit later seams.

## Qualified checkpoint and continuation

Source imports, root native builds, normal CLI execution/rendering, the first
neutral mechanics ownership split and all current regression gates are complete.
This is a staged restructuring checkpoint, not complete FEA/MBD independence.
The next code change should follow `NEXT_SEAMS.md`: remove the generic visual
model's FE implementation dependency before extracting participant/system services.
Do not reopen completed imports or rerun the 11-hour physical trajectory merely
to resume this migration. `OPERATING.md` records tested operator entry points.

## Retained demonstrations and first publication

The original spring, NSC collision/mixer and SCM lugged-wheel examples now build
and execute through native Bazel targets. Each completed six seconds headlessly
and with recorded VSG visualization; all model telemetry matched exactly between
the two modes. All videos passed full decoding and visual review. These examples
use CPU dynamics and GPU graphics; they do not claim a new CUDA MBD/SCM port.

A shared capture utility preserves the existing Yaris replay: its short regression
MP4 remains byte-identical after the extraction. The final product/example suite
passed 24 test targets. The selected Robodyna logo is in `assets/brand/`.
The next architecture seams remain in `NEXT_SEAMS.md`; demo qualification did not
complete standalone FEA/MBD dependency separation.
