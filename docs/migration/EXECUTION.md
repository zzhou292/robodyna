# Active restructuring plan

The source integration began on `work/initial-integration`. The user subsequently
authorized implementation of the Chrono API naming plan with regression tests and
preserved license notices. Current work is on `work/robodyna-api-renaming`;
destination is https://github.com/zzhou292/robodyna.git. No publication of this
rename series has been requested.
The user subsequently authorized initial publication to this repository after
retained Chrono demonstrations and a Robodyna logo are complete. Those tasks
passed their runtime/media gates and publication completed. The subsequent branded
README/media checkpoint is `26ef28a9d68adf9d170a78ef1e796bff4ef09d30`.

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
4. API/MODULE MIGRATION ACTIVE — neutral and generic visual owners are qualified,
   and actual Body/Mesh/System families now use canonical Robodyna definitions.
   The current integrated gates are recorded below. Independent full FEA/MBD
   closures and the richer coupled beam/body gate remain pending.
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

## Completed neutral extraction and remaining seams

The first dependency split now owns the existing moving-frame and inertia
implementations under neutral mechanics, with core/archive and variable-block
support. Its standalone wrench and inertia gates passed without the aggregate.
This changed ownership, not equations. The inherited include and archive names
remain intact at this checkpoint.

Source review also found a visualization dependency through `ChObj` and
`ChVisualShapeFEA`. Moving the entire participant hierarchy without resolving that
dependency would hide FEA inside a supposedly neutral package. Treat visual adapters,
mesh/system services, contact reporting and mixed assembly as explicit later seams.

## Qualified checkpoint and continuation

Source imports, root native builds, normal CLI execution/rendering, the first
neutral mechanics ownership split and all current regression gates are complete.
This is a staged restructuring checkpoint, not complete FEA/MBD independence.
The requested naming migration adds the N0/N1 API/fixture batch described below
as the next proposed code work. The next dependency extraction still follows
`NEXT_SEAMS.md`: remove the generic visual model's FE implementation dependency
before extracting participant/system services. Its design can proceed in parallel
with the API batch; keep their executable changes separately reviewed.
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

## Active Robodyna API naming migration

The current verified checkpoint implements the initial public API and the first
real definition families on `work/robodyna-api-renaming`. No remote publication
is authorized for this series. Original Chrono notices and exact BSD license text
remain; OpenRadioss removal is deferred.

The following are actual definitions, with reverse aliases in legacy headers:

- `robodyna::mechanics`: mass properties, inertia utilities and composite inertia.
- `robodyna::mbd`: Body, BodyAuxRef and all eight Easy-body types.
- `robodyna::fea`: Mesh.
- `robodyna::simulation`: System, SystemNSC and SystemSMC, including compatibility
  for the nested SMC force-algorithm interface.

These source families live in their owning directories and compile exactly once.
The domain implementations still use the explicit mixed backend; namespace/file
moves do not establish independent FEA/MBD libraries. The first neutral ownership
split and generic visualization/object split are qualified, including the explicit
FE attachment adapter. The remaining services/assembly/contact/load boundaries
are described in `NEXT_SEAMS.md`.

Maintained capture/replay consumers now use canonical Body, Simulation and mapped
value APIs. Pointer-only System contracts use its thin public forward target.
The used public headers are installed by the retained CMake build. Stable archive
identities preserve the frozen JSON/XML/binary fixtures, including the captured
Linux/GCC unregistered System key. Rebuild native consumers/plugins for the new
C++ symbols; saved archive compatibility does not promise old binary ABI.

### Completed verification

- 49 native/product test targets passed, including CLI, viewer, demos, API probes,
  frozen archives, source ownership and declared binding generation.
- Three separate retained-CMake build/runtime targets passed.
- Three binding runtime/ELF targets passed: core and FE Python use one native
  backend; FE ran 1000 coupled steps (0.1 s), with zero attachment error, common
  time and consistent reactions. Both core native wrappers compiled. Managed C#
  runtime and full optional-module runtime remain unqualified.
- Seven explicit declaration views share one authenticated generator and registry.
  Four CMake language/FEA configurations and 13 incorrect-pin rejection/recovery
  checks passed. Historical core/FE/vehicle generated APIs match. Only proven
  diagnostic source locations are normalized; raw failed receipts are retained.
- A fresh 101-step GPU Yaris run passed. All 50 archive files, viewer input and
  every non-timing summary field match the pre-rename baseline exactly. Full C++
  replay passed; physical timestep remains 150 ns and numerical settings unchanged.
- The three original six-second spring/NSC/SCM summaries match exactly. Rebuilt
  rendering also matches both saved PNGs, frame index and fully decoded MP4 byte
  for byte; the final frame was inspected visually.

`RENAME_QUALIFICATION.json` records the receipts, hashes, resources and exact
boundaries. `SOURCE_TRANSFORMATIONS.json` contains 77 reviewed inverse recipes
without repinning original source. Parser ledgers hold only their owning public
header recipes; complete implementation history remains in the global ledger.
Existing archive/partial-binding limitations remain in `PREEXISTING_ISSUES.md`.

### Next execution batches

1. Qualify the staged mixed-assembly lifecycle baselines, then move the actual
   Assembly family with its ADL swap, old archive identity and ownership rules.
   Preparation is in the outer `crash-work/staging/rename-assembly-1` directory;
   it has not executed or changed production Assembly code.
2. Extract the specific system services needed by FE and rigid participants,
   then mixed assembly, contact reporting and coupled load boundaries. Require
   actual independent-domain link/runtime gates and the richer beam/body test
   before declaring full FEA/MBD separation.
3. Continue coherent shared-frame/participant, math, node/element, material and
   numerical naming families. Freeze affected identities before their moves.
4. Finish application-owned directory ownership, public binding package naming,
   documentation input roots and optional-module qualification from the capability
   matrix. Retain compatibility paths while those capabilities remain pending.

Run native and retained CMake compilation in separate guarded phases. Native
builds use at most four compiler workers; large wrapper units use one. GPU and
render commands manage their own guard. Preserve frozen runs and failed receipts,
keep other GPU jobs running, and do not repeat the 100 ms physical trajectory for
name-only changes. No numerical optimization or OpenRadioss removal belongs in
this naming series.
