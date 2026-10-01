# Chrono to Robodyna: public API and implementation migration

Status: implementation active on `work/robodyna-api-renaming`. Public API,
archive identity, inertia, generic visualization, actual Body/AuxRef/Easy/Mesh/
System families, maintained consumers and named core/FE binding profiles are
qualified, including the mixed Assembly definition. Remaining domain service
boundaries and the branded time/IO forwards are next. Audited repository
baseline: `26ef28a9d68adf9d170a78ef1e796bff4ef09d30`.

Current scope: execute the Chrono-to-Robodyna naming migration. OpenRadioss removal
is explicitly deferred by the user; the boundary discussion below records future
work and does not authorize source, fixture or attribution removal in this series.

This plan implements the requested `chrono::ChBody` →
`robodyna::mbd::RbBody` and `chrono::fea::ChMesh` →
`robodyna::fea::RbMesh` direction. It extends, rather than replaces,
[MODULES.md](../architecture/MODULES.md) and [NEXT_SEAMS.md](NEXT_SEAMS.md).
The goal is a first-party Robodyna implementation with a small, explicit legacy
compatibility surface. Aliases are an intermediate delivery, not the endpoint.

## 1. Measured scope and current limits

[CHRONO_RENAME_SCOPE.json](CHRONO_RENAME_SCOPE.json) records the baseline,
selection rules, exclusions, and per-module counts. These are lexical counts,
including comments and strings, not a semantic count of exported APIs. Bundled
third-party code and embedded Bullet/GImpact are excluded from the imported census.

| Measurement | Observed |
| --- | ---: |
| Imported source/interface files scanned | 3,767 |
| Files containing `Ch` followed by an uppercase letter | 3,596 |
| Such token occurrences | 123,955 |
| Distinct class/struct spellings detected, including forward declarations | 1,098 |
| Files named with a `Ch` prefix | 1,883 |
| Files referring to `ChBody`, including selected consumers | 710 |
| Files referring to `ChMesh`, including selected consumers | 157 |
| Files referring to `ChSystemNSC`, including selected consumers | 301 |
| Application files containing `Ch…` tokens | 124 |
| TL-FEA files containing `Ch…` tokens | 13 |

The last two counts also include provenance comments. TL's production kernels
mostly already use their own types; apparent Chrono references there are not a
reason to rewrite them. The renderer and application adapters are actual Chrono
API consumers. The application also has its own `chrono/AcceptedReplayScene.h`
and related include paths: these are distinct from the imported Chrono headers.

Compile impact is wider than the files with direct matches because headers expose
these types transitively. The native mechanics build still owns 482 original
translation units: 21 compile in extracted neutral libraries, and 461 remain in
the combined compatibility implementation. A renamed header does not establish
independent MBD or FEA linkage.

All Chrono modules are retained, but only the named build/runtime gates in
[CAPABILITIES.md](CAPABILITIES.md) are qualified. Optional SDKs, Python/C# binding
builds, full vehicle subsystems and independent FEA/MBD builds remain separate work.

The subsequent body checkpoint qualifies the optional core Python runtime and
both native binding wrappers, with one shared backend. It does not qualify
managed C# execution or every optional module. See `RENAME_QUALIFICATION.json`.
The current plan preserves explicit aggregate dependencies while real types move;
full domain independence still requires the service/contact/assembly seams in N4.
This ordering allows source-compatible implementation progress without pretending
that the existing domain cycles have disappeared.

## 2. Naming decisions

Use `robodyna` as the C++ root namespace, lower-case responsibility namespaces,
and `Rb` for inherited `Ch` type prefixes. Preserve meaningful suffixes such as
`NSC`, `SMC`, `ANCF`, `QEPH`, `xyzrot`, and template scalar parameters. A branding
change must not imply a new formulation or backend.

| Existing API | Proposed canonical API | Owner |
| --- | --- | --- |
| `chrono::ChBody` | `robodyna::mbd::RbBody` | Rigid body |
| `chrono::ChBodyAuxRef` | `robodyna::mbd::RbBodyAuxRef` | Rigid body with auxiliary reference |
| `chrono::ChLinkTSDA` | `robodyna::mbd::RbLinkTSDA` | Multibody spring/damper |
| `chrono::fea::ChMesh` | `robodyna::fea::RbMesh` | FE mesh |
| `chrono::fea::ChNodeFEAxyz` | `robodyna::fea::RbNodeFEAxyz` | FE node |
| `chrono::fea::ChElementSpring` | `robodyna::fea::RbElementSpring` | FE element |
| `chrono::ChSystem`, `ChSystemNSC`, `ChSystemSMC` | `robodyna::simulation::RbSystem`, `RbSystemNSC`, `RbSystemSMC` | Mixed mechanical composition and stepping |
| `chrono::ChAssembly` | `robodyna::simulation::RbAssembly` | Mixed bodies/links/shafts/meshes |
| `chrono::ChBodyFrame` | `robodyna::mechanics::RbBodyFrame` | Shared moving-frame interface, also used by FE nodes |
| `chrono::ChPhysicsItem` | `robodyna::mechanics::RbPhysicsItem` | Shared mechanical participant |
| `chrono::ChMassProperties`, `ChInertiaUtils` | `robodyna::mechanics::RbMassProperties`, `RbInertiaUtils` | Neutral inertia utilities |
| `chrono::ChVector3d`, `ChQuaterniond`, `ChFrame<>` | `robodyna::core::RbVector3d`, `RbQuaterniond`, `RbFrame<>` | Math/value types |
| `chrono::ChSolver*`, `ChTimestepper*` | `robodyna::numerics::RbSolver*`, `RbTimestepper*` | Numerical algorithms; dependencies audited by family |
| `chrono::ChContactMaterialNSC/SMC` | `robodyna::contact::RbContactMaterialNSC/SMC` | Distinct contact law parameters |
| `chrono::ChTriangleMeshConnected` | `robodyna::geometry::RbTriangleMeshConnected` | Geometry; distinct from an FE mesh |
| `chrono::vsg3d::ChVisualSystemVSG` | `robodyna::visualization::vsg::RbVisualSystemVSG` | Rendering backend |
| `chrono::vehicle::SCMTerrain` | `robodyna::vehicle::SCMTerrain` | Keep the descriptive non-branded type name |

`RbSystemNSC` belongs above MBD and FEA. Putting it inside `mbd` would misrepresent
its existing ability to assemble flexible and rigid participants together.
Do not create two system clocks to obtain separate namespaces.

This is primarily a type/namespace change. Ordinary methods such as `SetPos`,
`SetMass`, `AddBody` and `DoStepDynamics` retain their names and behavior. Explicitly
branded functions get individual mappings: for example, propose `GetTime` and
`SetTime` for `GetChTime` and `SetChTime`, with forwarding legacy methods. Audit
inherited overloads and binding names before adding these. Expose data-path helpers
through Robodyna's IO API while preserving the existing single data-path state.
Do not prepend `Rb` to every method or change member field names unnecessarily.

Namespace placement follows ownership, not a file's old directory or spelling.
Types without a `Ch` prefix retain their descriptive names unless a concrete
collision or usability problem justifies another rename. Constants, registration
macros and export macros need explicit mappings rather than a global `CH` rewrite.

## 3. Public headers and one implementation

Proposed public include paths are short and module-specific:

```text
include/robodyna/core/RbVector3.h
include/robodyna/mechanics/RbBodyFrame.h
include/robodyna/mbd/RbBody.h
include/robodyna/fea/RbMesh.h
include/robodyna/simulation/RbSystemNSC.h
include/robodyna/contact/RbContactMaterialNSC.h
include/robodyna/visualization/vsg/RbVisualSystemVSG.h
```

Public headers are the API entry points; implementation files and owning BUILD
targets live in `src/<owner>/`. Internal callers use explicit module dependencies.
Avoid an everything-included `Robodyna.h` or a second forwarding class hierarchy.

The first usable API can be a direct type alias:

```cpp
// Proposed include/robodyna/mbd/RbBody.h
#include "chrono/physics/ChBody.h"
namespace robodyna::mbd {
using RbBody = ::chrono::ChBody;
}
```

This has the same C++ type, shared-pointer conversions, RTTI, object layout,
registration and arithmetic as the existing implementation. It creates no runtime
adapter. It does not yet change the mangled binary symbol or remove legacy names
from compiler diagnostics. Do not create `class RbBody : public ChBody` as branding.

After the implementation family moves, reverse the alias direction:

```cpp
// Legacy chrono/physics/ChBody.h compatibility header, after migration
#include "robodyna/mbd/RbBody.h"
namespace chrono {
using ChBody = ::robodyna::mbd::RbBody;
}
```

Each family has one canonical definition and one compiled owner at every stage.
Do not compile both old and moved source files. Compatibility headers must never
cycle back into themselves. Old forward declarations, friend declarations,
elaborated type specifiers, explicit template specializations and instantiations
must be audited: a former `class ChBody;` cannot coexist with a `using ChBody = …`.
Use small canonical forward-declaration headers and explicit legacy aliases.

Namespace moves also affect argument-dependent lookup, free operators, qualified
calls, traits and SWIG specializations. Move operators with their canonical types;
retain deliberate legacy using-declarations where needed. Do not add global using
directives to conceal missing dependencies.

During the alias phase, `//include/robodyna/<module>:api` targets may explicitly
depend on the current combined backend. Mark that transitional dependency. Switch
to actual `//src/<module>` owners as they qualify. Never label a facade target an
independent domain until its real transitive compile/header/link closure passes.

## 4. Compatibility contract before real renames

| Boundary | Required behavior |
| --- | --- |
| Existing C++ source | Legacy headers/types remain during migration; test mixed old/new callers |
| Public Robodyna source | New examples use canonical headers/names without directly spelling `chrono` |
| ABI | Aliases initially preserve types; real namespace/class changes break ABI and require a coordinated rebuild of libraries, consumers, plugins and generated bindings |
| Stored class and field IDs | Preserve old wire names and layouts independently of C++ spelling |
| Class factory | One registry and one canonical serialization tag per type; optional read aliases do not change writer identity |
| Physics and CUDA state | Preserve formulations, data layout, flags, stream ownership, step policy, assembly order and accepted/trial lifetimes |
| Protocols and assets | Preserve existing JSON/schema IDs, model file keys, communication wire names and asset lookup behavior |
| History and licenses | Preserve source pins, inherited notices, component licenses, model attribution and frozen qualification evidence |

Concrete source hazards:

1. `CH_FACTORY_REGISTER` stringizes its argument into the saved class tag
   (`chrono/core/ChClassFactory.h:548`). `ChBody` and `ChSystemNSC` use it.
   `ChArchiveJSON.cpp:228` writes that name as `_type`.
2. `CH_FACTORY_REGISTER_CUSTOMNAME` does **not** solve this: its second argument
   names the registration variable, while the stored tag is still `#classname`
   (`ChClassFactory.h:553`). Implement an explicitly named tag facility or reuse
   the underlying registration constructor with stable tags.
3. Registering old and new tags naively for one type overwrites the single RTTI
   entry (`ChClassFactory.h:296`), and unregistering one can erase the other.
   Keep one canonical tag; implement separate read aliases only if needed.
4. Version keys fall back to `typeid(T).name()` for unregistered classes
   (`serialization/ChArchive.cpp:34` and `:55`). Pin the existing keys of migrated
   archived types, including abstract bases and concrete template specializations.
   Capture legacy fixtures on the supported toolchain. Do not invent a promise
   that historically compiler-dependent archives were portable across all compilers.
5. `CH_UPCASTING` stringizes both types. Preserve conversion graph identities and
   the correct adjusted pointer for every base of multiply inherited objects.
6. `CHNVP` can derive a field key from the source expression
   (`serialization/ChArchive.h:384`). Keep field names or pass explicit old keys.
7. Header guards, export macros, generated configuration, symbol visibility, Eigen
   plugin include order and static registration retention belong to the build
   contract. A source-compatible header is insufficient if factory-only classes
   disappear from the link.

Paths in this numbered list are beneath `src/compatibility/chrono/src/`.
Keep ordinary object archive behavior distinct from Robodyna's visualization
archives: neither establishes physical restart support. Preserve the existing
limitation that `ChVisualModel` archives ordinary shapes but not its FE shape list.

## 5. Implementation milestones

### N0 — Freeze the rename contract and reference fixtures

This probe and module proposal are complete. Before edits to executable code:

- Create a reviewed symbol manifest containing old fully qualified name, canonical
  name/header/owner, legacy include, archive identity, binding exposure and migration
  state. Use semantic declaration identity where available; lexical counts are
  discovery aids, not an automatic rewrite specification.
- Start with a small complete API family, including dependencies needed to write
  a body/spring and FE mesh example. List exact qualified symbols before editing.
- Freeze small JSON/XML/binary fixtures from the current implementation for
  representative values, registered classes, abstract bases and pointer graphs.
  Establish what each supported archive format actually retains.
- Record baseline factory-only creation, multiple-inheritance pointer conversions,
  shared identity, versions, and existing physics telemetry. Reuse existing fixtures
  and receipts where they already cover the requirement.

Exit: explicit naming/ownership map and reproducible compatibility oracles. No
new simulation formulation and no large vehicle run are required.

### N1 — Deliver a useful Robodyna C++ API

Add narrow public headers with aliases for the initial closure: core vectors,
quaternions/frames, shared frame/participant types as needed, body/easy-body/spring,
mesh/node/spring element, NSC/SMC systems, contact materials and the solver/timestepper
types used by the examples. The approved symbol manifest bounds this batch.

Add separate Bazel header/API targets. Compile standalone public headers and
old/new includes in both orders. Verify `std::is_same`, shared-pointer interoperability,
virtual dispatch and factory construction. Port the small rigid and FE examples
to canonical includes/types; retain legacy consumer tests.

Exit: runnable Robodyna-named body and FE examples, old callers still working,
unchanged numerical telemetry, and no wrapper allocation/virtual-call overhead.
Implementation names are still inherited; state that explicitly.

### N2 — Decouple archive and registry identities from C++ names

Add the minimum explicit identity support in the existing factory/archive layer.
Keep old writer tags, field keys and version identities. Audit all upcasting edges
in each migrated family. Preserve one process-wide factory across new and legacy
entry points. Avoid a new generic serialization framework.

Test old fixture → new reader; new writer → preserved reader where historically
supported; pointer identity, constructor data and shared/cyclic graphs where the
existing archive supports them. Test duplicate-tag rejection and deterministic
writer identity if read aliases are added. Keep malformed/unknown-type errors clear.

Exit: actual renamed types can preserve tested archive behavior. Registered and
RTTI-versioned families cannot cross to canonical definitions before this gate.

### N3 — Prove real implementation renaming on a neutral owner

Use the already extracted inertia family as the first bounded pilot:
`ChMassProperties` and `ChInertiaUtils` → their `Rb` names under `mechanics`.
Keep the useful non-branded `CompositeInertia` name. This family already has its
own compiling owner and behavioral tests, and does not require a whole-system move.

Separate mechanical symbol edits from physical file relocation. Reverse legacy
aliases, repair forward declarations/qualified uses, preserve target-local flags,
and update Bazel plus retained CMake source wiring in the same reviewed batch.
Verify exactly one implementation owner and legacy/new caller linkage.

Exit: one real canonical implementation family, not only a facade, with unchanged
inertia/frame results and no link to a second implementation. Repeat the pattern
for suitable neutral families; archive-bearing families additionally require N2.

### N4 — Finish dependency seams, then migrate domain implementations

Apply [NEXT_SEAMS.md](NEXT_SEAMS.md) in separate behavioral changes before combining
those changes with broad names:

1. Remove the generic visual model's FE implementation dependency using the planned
   explicit per-model FE updater. Preserve copy/share, owner, clear/re-attach and
   update-order behavior.
2. Extract narrow mesh environment/setup services from the concrete mixed system.
3. Separate neutral contact algebra from concrete body reporting/response adapters.
4. Separate FE-only loads/builders from concrete body/motor attachment helpers.
5. Keep `ChAssembly`/`ChSystem` mixed composition above domain owners.

Then migrate coherent dependency families: neutral core/mechanics/numerics, MBD
bodies and connections, FE nodes/elements/meshes, concrete coupling, and mixed
systems. Within cycles, migrate the smallest strongly connected family together
or explicitly retain a temporary edge; never hide a cycle with a new umbrella.

Exit: canonical `RbBody`, `RbMesh` and `RbSystemNSC` implementations; standalone
MBD and FE gates without opposite-domain implementations; and a flexible-beam/body
coupling test with the same common assembled solve, reactions and step clock.
Different namespaces alone do not satisfy this milestone.

### N5 — Migrate product consumers and bindings

Move application adapters, viewer, demo support and SCM consumers to canonical APIs.
Separate application-owned `chrono/` headers into their real visualization or
simulation owner. Preserve shared renderer utilities and source model asset paths.

Define `robodyna` Python and `Robodyna.*` C# package/module naming before regenerating
SWIG. Update hand-authored typemaps, `%extend`, shared-pointer directives, director
hooks, template specializations, C# multiple-inheritance helpers, native DLL/PInvoke
names and imports. A C++ alias is not proof of a usable Python/C# API. Keep optional
`pychrono` compatibility as a shim to the same binding runtime if supported, not a
second independently registered native type system.

Include the SolidWorks/Python importer in that boundary: the existing parser
accesses SWIG native pointer storage as a `shared_ptr<ChPhysicsItem>`
(`chrono_parsers/ChParserPython.cpp:297`). Test a retained exported script adding
objects to a new-named system, plus cross-module callbacks, rather than treating
successful package import as sufficient compatibility.

Exit: actual import/build/runtime gates for each binding claimed supported, with
constructor, ownership/lifetime, inheritance and serialization coverage. Missing
SDK environments remain named pending capabilities, not silent deletions.

### N6 — Extend to every retained Chrono capability

| Family | Destination and additional gate |
| --- | --- |
| Vehicle, models, robotics, SCM | `vehicle`, model catalogs, `robotics`; rigid/FE tire and model loading cases where supported |
| SPH/FSI and TDPF | `sph`, `potential_flow`, coupling; GPU/host boundary state and coupled stepping |
| DEM and multicore | `dem` and explicit numerical/execution backends; preserve distinct algorithms and settings |
| Modal and peridynamics | `analysis::modal`, `peridynamics`; preserve mixed assemblies and declared FE-node dependencies |
| Sensors | `sensor`; OptiX/CUDA configuration, callback and data-transfer regression |
| CAD/parsers/postprocess | `io` and `postprocess`; format and model-loading fixtures |
| VSG/Irrlicht | `visualization`; geometry/color/capture parity and optional SDK builds |
| MUMPS/Pardiso | `numerics` backend packages; solve residual and SDK linkage |
| ROS/FMI/SynChrono/preCICE | `integrations`; preserve external protocol names and actual transport/export tests |

Qualify optional families one at a time using [CAPABILITIES.md](CAPABILITIES.md).
Retain compatibility facades while required external environments are unavailable.
Do not mechanically alter generated FlatBuffers or other protocol output: preserve
schema/wire identity and change its owning generator/input only when necessary.

TL's eventual `tlfea` → `robodyna` implementation migration is a separate bounded
series. It must preserve CUDA annotations, layouts, stream/lifetime contracts,
explicit and implicit/ANCF paths, and its qualified FE/rigid/CIN state owner. The
Chrono type rename must not replace those kernels with similarly named CPU types.
Do not merge distinct Chrono/TL math or solver implementations merely to standardize
spelling. Existing `robo_dyna.*` stored schemas remain stable.

### N7 — Make the new API the default and retire legacy implementation ownership

Switch maintained examples, operator docs and generated API docs to Robodyna.
The retained Doxygen input currently scans its imported `../src` tree; add the
canonical public include and relocated implementation roots in the owned docs
configuration before claiming generated documentation covers the migrated API.
Offer legacy headers/labels in a documented compatibility package; deprecate them
only after internal callers migrate and optional capabilities have a tested path.
Keep the shim until an explicit compatibility policy permits removal.

Exit: migrated implementations reside with their real owners; compatibility owns
only deliberate adapters/aliases; all required functionality has declared build and
runtime status. An allowlisted source check blocks new legacy API use in migrated
code without rejecting preserved notices, tests, external protocols or fixtures.

The exclusion manifest should explicitly include YAML `chrono-version` and thread
key `chrono`, DDS `SynDDSMessage`, standard ROS message names, FMI C exports and
preCICE mesh/data identifiers. Exported Blender scripts and their importer must
retain a compatible contract. Sensor kernel/program names passed as strings must
agree with the corresponding compiled device entry points. Qualify these contracts
with their real optional backends, not only the host mechanics build.

## 6. Tests and acceptance scale

| Gate | Required checks | When |
| --- | --- | --- |
| Public API | Header self-containment, both include orders, old/new callers, type equality during alias phases, overload/ADL/template cases | Every family |
| Ownership | Actual transitive headers/link owners, exactly one compiled implementation, no opposite domain behind a supposedly standalone target | Every owner move |
| Factory/archive | Factory-only retention, stable tags/versions/fields, adjusted base pointers, shared identity, historical fixtures | Every affected registered/archived family |
| Neutral mechanics | Inertia, transforms, virtual frame dispatch, wrench/power invariance | Core/mechanics changes |
| MBD/FE/coupling | Existing rigid and FE spring checks; independent domains; coupled beam/body constraints and common clock | Domain/system changes |
| Product | Source-port checks, importer tests, CLI/replay tests; existing 101-step CUDA baseline when runtime consumers or layouts change | Product integration checkpoints |
| Media | Existing short accepted-archive replay, part colors and capture behavior; compare telemetry before visual approval | Viewer/SCM changes |
| Bindings/optional SDKs | Import, construct, subclass/callback where supported, pointer ownership and module-specific smoke | Each newly claimed profile |

Reuse `//tests/chrono:native_host_tests`,
`//src/mechanics/kinematics:frames_test`, `//src/mechanics/inertia:inertia_test`,
existing source-owner tests and the qualified spring/NSC/SCM telemetry before
inventing parallel harnesses. New boundary tests belong near their owning module.

For name-only arithmetic-preserving work, require the current deterministic
numerical outputs; normalize only deliberately changed diagnostic/source labels.
Do not loosen numerical tolerances to pass a branding change. Retain the 101-step
byte comparison as a product integration gate where applicable. A short gate is
not a reason to regenerate the preserved 100 ms trajectory; reuse that archive
for rendering. Investigate divergence before making any longer run necessary.

## 7. Build, provenance and review procedure

- Root Bazel remains canonical. Keep CMake wiring usable for retained sources,
  with explicit source-list updates at moves; do not create a second package graph.
- Preserve PCH/Eigen plugin ordering, `CH_STATIC`/visibility semantics and target
  arithmetic flags. New `RB_*`/`ROBODYNA_*` macros need scoped compatibility adapters
  and tests before old macros are removed. CMake package/target compatibility and
  generated `ChConfig.h`/`ChVersion.h` paths are a separate reviewed build batch.
- Preserve `SOURCES.json` and original import trees as historical evidence. Current
  source/hash checks must evolve through a reviewed migration ledger containing
  old/new paths, hashes, owning targets and verification; do not simply overwrite
  import hashes or bless changed bytes. Update the live neutral ownership checks
  when definitions move; retain the original 21-file/482-file checkpoint evidence.
- Never replace `std::chrono`, third-party symbols, copyright text, original commit
  references, asset licenses, protocol identifiers or immutable receipts. Preserve
  component licensing; a rename does not change the repository's mixed-license scope.
- Use AST-aware edits for compiled profiles plus explicit reviewed mappings for
  CUDA/SWIG/macros and unqualified optional sources. Do not assume one compile
  database covers every optional module. Review constructors/destructors, casts,
  friends, templates, export decorations and generated inputs in each batch.
- Separate commits for compatibility support, symbol changes, file moves and
  dependency refactoring. Do not mix numerical fixes, GPU optimization, compiler
  flag changes or large formatting changes into rename commits.
- Parallelize independent source audits and family edits. Serialize qualification
  under existing workstation guards; this scope does not require larger resources.

At each milestone, review the public header set, real dependency graph, remaining
legacy uses, optional-module status and evidence. Replan if a family introduces a
cycle, needs an unsupported SDK, or changes archive behavior. Keep progress as
separate counts for mapped APIs, usable aliases, actual implementation moves,
independent owners and qualified capabilities; a single rename percentage would
hide those differences.

## 8. OpenRadioss boundary

Deferred work: retain the current implementation, reference fixtures and notices
throughout the naming migration. Revisit removal as a separate scoped change.

The product must not require an OpenRadioss solver checkout, executable, library or
Fortran toolchain to build and run its Yaris profile. The current implementation
uses native C++/CUDA ports inside the absorbed TL sources. For example,
`lib_src/collision/BUILD.bazel` owns TYPE25 C++/CUDA search, force and transaction
targets, and the production app depends on them through `@legacy_fea`.
The prefix is provenance/formulation naming, not a call to an external solver.

There are three distinct categories:

| Category | Treatment |
| --- | --- |
| Full upstream checkout and CPU executables for comparison | Keep as optional external development tools, outside the product source/build/runtime closure |
| Pinned original Fortran snippets, source manifests and oracle harnesses | Verification-only; keep minimal reproducible fixtures or move them to a separately admitted, hash-pinned test-data bundle |
| Production C++/CUDA ports of contact, elements and material behavior | Keep as Robodyna implementation under its owning modules, with retained source attribution and license notices |

The repository has selected reference source under
`src/fea/legacy/lib_utest/qualification/*/native/original/`; that is different
from embedding and invoking the full OpenRadioss solver. Do not delete those
fixtures until their replacement location is pinned and the owning verification
targets pass. A codebase with no external solver dependency is achievable without
discarding the independent checks that established the ports' correctness.

The source audit reran `python3 -m build_defs.app.check_port`: all 95 production
application libraries and 473 source files passed its static source/dependency
checks, including the rejection of direct Fortran-oracle/GTest mappings. That check
is not a complete transitive dependency proof. Add a build-graph gate on the actual
product closure before any reference-data relocation: no oracle targets, `.F`/`.F90`
compile inputs, OpenRadioss binaries/libraries or reference checkout paths in the
production executable/runfiles. Test in a staged environment with those optional
tools absent, without moving or deleting preserved workspace evidence.

A separate read-only traversal of literal production dependencies reached 49
initial TL targets and 270 transitive TL targets (247 `cc_library`, 23
`cuda_library`), with no unresolved dependency declarations or test/oracle targets.
The reached explicit external library dependency was Eigen; CUDA support comes
through the toolchain. This corroborates the runtime separation at the source
declaration level, but does not replace the proposed configured build/runfiles gate.

There is still an input-preparation dependency to track: the qualified Yaris profile
requires a pinned `native_v6_raw8_heph_explicit_cin28` control packet
(`src/simulation/driver/native/Prepare.cpp:26` and
`src/compatibility/app/case/vehicle_native_contact/source/Inputs.cpp:28`). Running
the prepared case reads that artifact; it does not launch the reference solver.
Do not claim arbitrary new model import is fully independent until the provenance
and generation of all required preparation artifacts are audited and equivalent
first-party importer/setup paths are qualified. Keep this importer milestone
separate from renaming the numerical implementation.

`modelio/solid_control_packets/NativePacketSource.h:18` describes the offline
import, and `Reader.cpp:31` requires the `offline_actual_native_packet_export`
provenance tag (paths beneath `src/compatibility/app/`). Preserve that evidence
when moving reference tooling out of the production-facing layout.

Removing an external reference checkout does not remove the production ports'
OpenRadioss derivation or their existing AGPL-3.0-or-later notices. Do not describe
this as an independently authored or wholly BSD implementation after a rename.
No OpenRadioss-related source or fixture was removed during this probe.

## 9. Recommended immediate execution batch

Implement N0 fixtures and N1 public aliases first, with a small reviewed symbol
manifest and two runnable examples. In parallel, design N2 identity handling and
prepare the existing visual dependency seam; keep executable changes separate.

The first result should let a user include `robodyna/mbd/RbBody.h`,
`robodyna/fea/RbMesh.h`, and `robodyna/simulation/RbSystemNSC.h`, build from the root
Bazel graph, and run unchanged mechanics. Next prove the real inertia-family move,
then carry the same method through body/mesh/system ownership. Broader timelines
should be estimated after these two batches reveal binding and dependency costs;
the full 3,596-file surface is not one safe search-and-replace operation.
