# Active restructuring plan

The source integration began on `work/initial-integration`. The user subsequently
authorized implementation of the Chrono API naming plan with regression tests and
preserved license notices. Current work is on `work/robodyna-api-renaming`;
destination is https://github.com/zzhou292/robodyna.git. The user authorized
publication of the rename series; checkpoint `bcf1753` was pushed to the same
remote branch. The current follow-up replaces presentation branding and audits
retained demo/test exposure.
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

## Transparent branding and inherited example audit

The current follow-up replaces the opaque logo with a real RGBA asset, retires
dedicated Chrono logos from the working tree, and captures all five README views
again with the logo supplied by the native renderer. The user's review exposed
an inherited `ChLinkTSDA demo` window title; the presentation audit now includes
window captions, viewer overlays, startup labels and documentation navigation.
Required copyright notices and truthful backend/API references remain.

This presentation pass does not finish the C++ definition migration. Only the
qualified definition families listed below have moved; remaining Chrono API,
binding and compatibility paths must continue through their separate tests.
Do not describe the entire naming migration as complete.

The [demo/test audit](../verification/CHRONO_DEMOS_TESTS.md) confirms retention of
all 739 files in the pinned demo and unit-test trees, plus the separate FMI
example. Its catalog exposes source locations, existing CMake gates, actual root
Bazel targets and pending cases. Most inherited cases still lack root Bazel
targets; the empty inherited GoogleTest checkout also disables that CMake test
tree. Follow the report's module-by-module exposure sequence after this media
checkpoint instead of silently claiming the complete inherited suite passed.

This checkpoint is complete: 25 dedicated Chrono branding assets were retired,
547 exact inverse source histories verified, and all 16 focused Bazel test targets
passed, including real-image alpha validation. The three fresh demo captures
preserve every physics/control summary field; only the authenticated presentation
source hash changes. Both Yaris views replay the unchanged 100 ms archive. All five
delivery videos passed full decoding and anonymous CDN byte verification. The
[qualification record](../verification/TRANSPARENT_BRANDING.json) gives receipts,
scope and optional-runtime limitations. The next functional integration step is
the catalog's module-by-module demo/test exposure sequence; deeper API/module
migration remains governed by the separate checkpoints below.

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

## Earlier integration checkpoint

Source imports, root native builds, normal CLI execution/rendering and the first
neutral mechanics split were qualified before the API migration. The generic
visual separation has since passed too. Current naming/service progress and its
remaining boundaries are recorded in the active section below and NEXT_SEAMS.md.
Preserve the earlier evidence; neither a source import nor a namespace change
establishes independent FEA/MBD execution. OPERATING.md records the operator entry
points, and the retained 100 ms trajectory needs no rerun for naming changes.

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
real definition families on `work/robodyna-api-renaming`. Remote publication
was authorized and completed for checkpoint `bcf1753`. Original Chrono notices and exact BSD license text
remain; OpenRadioss removal is deferred.

The following are actual definitions, with reverse aliases in legacy headers:

- `robodyna::mechanics`: mass properties, inertia utilities and composite inertia.
- `robodyna::mbd`: Body, BodyAuxRef and all eight Easy-body types.
- `robodyna::fea`: Mesh.
- `robodyna::simulation`: System, SystemNSC, SystemSMC and Assembly, including compatibility
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

- 57 native/product test targets passed, including CLI, viewer, demos, API probes,
  frozen archives, source ownership and declared binding generation.
- Three separate retained-CMake build/runtime targets passed.
- Three binding runtime/ELF targets passed: core and FE Python use one native
  backend; FE ran 1000 coupled steps (0.1 s), with zero attachment error, common
  time and consistent reactions. Both core native wrappers compiled. Managed C#
  runtime and full optional-module runtime remain unqualified.
- Eight explicit declaration views share one authenticated generator and registry.
  Four CMake language/FEA configurations and 15 incorrect-pin rejection/recovery
  checks passed. Historical core/FE/vehicle generated APIs match. Only proven
  diagnostic source locations are normalized; raw failed receipts are retained.
- A fresh 101-step GPU Yaris run passed. All 50 archive files, viewer input and
  every non-timing summary field match the pre-rename baseline exactly. Full C++
  replay passed; physical timestep remains 150 ns and numerical settings unchanged.
- The three original six-second spring/NSC/SCM summaries match exactly. Rebuilt
  rendering also matches both saved PNGs, frame index and fully decoded MP4 byte
  for byte; the final frame was inspected visually.

`RENAME_QUALIFICATION.json` records the receipts, hashes, resources and exact
boundaries. `SOURCE_TRANSFORMATIONS.json` contains 84 reviewed inverse recipes
without repinning original source. Parser ledgers hold only their owning public
header recipes; complete implementation history remains in the global ledger.
Existing archive/partial-binding limitations remain in `PREEXISTING_ISSUES.md`.

### Next execution batches

1. Assembly and the four-method participant-service seam are now qualified.
   The interface owns no state; participants retain their original raw owner.
   Its declaration has a verified foundation-only boundary while its implementation
   remains in the explicitly mixed backend. Next, design and qualify the thin
   System storage/header boundary with the protected-access and Multicore
   collection-address obligations documented in NEXT_SEAMS.md.
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

The Assembly checkpoint also repeated the GPU archive, short-render and six-second
demo comparisons successfully. All recorded physics fields, both PNGs and the
short MP4 remain exact. Existing copy/assignment/batch quirks were preserved,
not repaired as part of the type migration.

Native branded-name forwards passed a 41-target gate. `GetTime`/`SetTime` delegate
to the existing System and object methods without updating other clocks or
advancing state. `robodyna/io/RbPaths.h` forwards to the existing three distinct
path stores and preserves literal concatenation and directory side effects.
Legacy binding exposure is intentionally retained; combined binding/CMake/product
qualification subsequently passed with the service interface.

The combined participant-service/time/IO checkpoint passed 57 native/product
targets, three separate CMake targets, three actual binding runtime/ELF targets,
the complete historical binding surface matrix and four configured install
profiles. A fresh GPU run still matches all 50 saved files and non-timing fields;
all six-second demo summaries and the short rendered frames/movie remain exact.
The service's extra System base requires rebuilding binary consumers, but no
new participant-side pointer, environment state, clock or physics formulation
was introduced. Full independent-domain implementation remains later work.
