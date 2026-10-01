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

The requested Chrono API rename has been probed against repository baseline
`26ef28a9d68adf9d170a78ef1e796bff4ef09d30`. Implementation is now authorized.
See [CHRONO_RENAME_PLAN.md](CHRONO_RENAME_PLAN.md) for module ownership, compatibility
risks, implementation milestones and tests; [CHRONO_RENAME_SCOPE.json](CHRONO_RENAME_SCOPE.json)
records the lexical inventory and its limitations.

The first active batch is a bounded public API and compatibility-fixture layer:
`robodyna::mbd::RbBody`, `robodyna::fea::RbMesh` and
`robodyna::simulation::RbSystemNSC`, plus the supporting types needed by small
working examples. Stable archive identities precede real registered-class renames.
The already extracted inertia family is the proposed first real implementation
rename; the visual/system-service seams still gate independent FEA/MBD ownership.

Parallel source work is bounded to public aliases/examples, archive fixtures and
identity support, and the inertia implementation pilot. Builds and qualification
remain serialized through the workstation guard. Freeze baseline archive fixtures
before changing production serialization. Preserve the original Chrono copyright
and author headers; relocated files reference `LICENSES/Chrono-BSD-3-Clause.txt`.

First naming checkpoint PASSED: 26 public alias headers, 52 independent include-order
compilation probes, both executed CPU spring examples, frozen JSON/XML/binary
archive compatibility, and the actual canonical inertia implementation. All 19
checkpoint test targets passed through native Bazel and the retained CMake bridge.
See `RENAME_QUALIFICATION.json` for receipts, scope and resource usage.

Second naming checkpoint PASSED: stable archive identities preserve both actual
pre-change object archives and a genuinely renamed test class with multiple
inheritance. Generic objects and visualization now own nine additional original
translation units without FE implementation dependencies, with FEA enabled. One
explicit FE attachment adapter preserves the existing geometry/update behavior.
All 21 native and three separate retained-CMake test targets passed.

Run native Bazel and foreign CMake compilation in separate phases. Use Bazel
`--jobs=4` for native builds; use `--jobs=1` when invoking the retained CMake bridge,
whose own build remains at four compiler workers. This prevents nested worker
pools from exceeding the total compiler allowance. The interrupted mixed attempt
and its complete cleanup remain recorded in `RENAME_QUALIFICATION.json`.

Third naming checkpoint: the actual body definition and implementation now live at
`include/robodyna/mbd/RbBody.h` and `src/mbd/bodies/RbBody.cpp`. Legacy headers alias
that single type. Original archive tags, four base conversions, constructor data,
child attachment and shared ownership pass the frozen pre-move fixtures.
The implementation still compiles in the explicitly transitional aggregate;
this is not independent MBD linkage.

Qualification passed 26 native targets, three separately built retained-CMake
targets, 25 product targets, and the 101-step GPU Yaris regression. Every one of
its 50 archive files, viewer input and non-timing summary fields matches the
pre-rename baseline exactly. The short run also passed full C++ replay. No
materials, equations, integration policy or timestep changed.

The optional core bindings now compile both native wrappers against one shared
backend. Python imports and exercises constructors, derived casts, containers,
child lifetime and inertia; ELF checks reject duplicated body/factory definitions.
The generated interface retains 949 public Python AST entries and 610 exact C#
files from the pre-inertia baseline. Managed C# execution is still unqualified.
SWIG uses a generated, source-authenticated legacy declaration view because its
parser does not preserve these aliases reliably; C++ uses the canonical type.
See `build_defs/bindings/README.md` and `RENAME_QUALIFICATION.json` for this boundary.

One inherited YAML-off link error was exposed by the binding runtime and fixed:
the unchanged `ChVisualShapeFEA::Settings::PrintInfo` definition is now available
under the same conditions as its declaration. A focused test covers that profile.
Other inherited limitations remain documented in `PREEXISTING_ISSUES.md`.

Body checkpoint is committed locally as `9581daa03f`. The optional CMake binding
probe also passed Python+C# and C#-only configuration, execution of the exact
configured SWIG commands, interface installation and rejection/recovery after a
copied contract pin changes. It does not claim a complete native wrapper build
through CMake; native wrapper compilation is covered separately by Bazel.

System archives are now frozen against that commit after two fresh producer
processes emitted identical bytes. All six focused fixture, runtime, shared
FE/body coupling, freezer and source-ownership targets passed. Next, migrate the
mesh and mixed-system definitions in separate batches.
Reuse the new source, archive and binding gates. Complete the remaining domain
service seams before claiming independent FEA/MBD libraries. Namespace changes
and physical ownership separation are tracked separately.

OpenRadioss remains an optional external reference solver, not a required product
runtime. Production C++/CUDA ports and their notices remain; verification-only
Fortran fixtures require a separately reviewed relocation if moved out of tree.
The user has explicitly deferred that removal work. Keep it outside the naming
migration; no source, fixture or notice removal is part of the next API batch.
