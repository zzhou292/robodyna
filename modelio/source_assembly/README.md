# Authenticated assembly startup inputs

The explicit [V3 layered elastic/plastic mode](LAYERED_SECTIONS.md) adds typed
host-only LAW1 admission without changing the V1 wall/archive boundary.
The explicit [V2 analytic/table LAW44 mode](ANALYTIC_LAW44.md) reuses these
same source and native-input modules. The default compiler and frozen V1
archives retain their table-only contract.

`SourceAssembly` reads the frozen `robo-dyna.source-assembly-inventory.v1`
artifact through the existing `output/ArtifactIO` byte cap and SHA-256
primitives. Its expected content identity is a required caller argument;
`PinnedYarisSixPartInventory()` names the retained 1,731,843-byte artifact
with SHA-256 `afbc9cc6b9cbbceec766e1aa106b548fcc7468ce1a0d902d5d7b69afb1873d00`.
Computing an expected hash from an arbitrary input is not authentication and
is used only for deliberate malformed-fixture tests.

The authenticated bytes are parsed once using RapidJSON full-precision,
iterative and encoding-validation modes. Duplicate keys, bounded nested
structure, source identities, scope flags, declaration associations and
complete node/parent coverage are checked before publishing immutable data.
`JsonReader` supplies shared typed field/card/block helpers; declarations,
geometry and attachments are separate modules. The original bytes remain
available for evidence that has no typed startup consumer yet, including
array hashes, full external incidence, raw source records and unresolved ties.

The retained six parts contain 1,030 original physical nodes and 915 original
parents: 804 QEPH quads and 111 native T3 triangles. Node and parent source IDs,
canonical indices, source lines, exact binary64 coordinates, complete raw
six-field shell records and repeated triangle fourth slots are preserved.
Parent traversal follows source part order, with independent QEPH/T3 indices.
Original ELFORM 2/16 metadata remains attached to each section and parent;
QEPH/C0 selection is the explicitly pinned OpenRadioss converter policy,
without an LS-DYNA formulation-equivalence claim.

The six material and section tables retain their source cards and two complete
curves (17 and 46 points). Reader limits bound the combined curve pool to 1,024
points. Native shell reference inputs use each parent's original density,
thickness, E and Poisson ratio. The material adapter requires the named
`OpenRadiossDirectImportDefault` rate policy: source VP=0/C/P are retained, and
the same audited direct-import resolution used by `SourcePartMaterial`
sets the LAW44 VP2/ISRATE1 filter cutoff to 10,000/s. The source MAT024 lacks a
direct Fcut field: the absent converted-model value becomes zero through
`CPP_GET_FLOATV_FLOATD`, then `HM_READ_MAT44` resolves ISMOOTH=1 to this value.
This resolution differs from exporting and re-reading a Radioss CFG default.
The converter pin and source hash remain in the assembly data; the previously
audited chain is documented in `case/source_part_plastic/SourcePartMaterial.cpp`.

Ten nodal-rigid records remain typed: six internal groups with 76 disjoint
members and four outgoing groups. Source set ordering and blank flags are
retained. The released frontier keeps all 13 outgoing spotweld records, 37
external node IDs, six external part IDs and the unresolved tied source scope.
External-only nodes never enter the selected physical-node input ranges.

`SourceAssemblyShellInput::input()` returns borrowed TL
`ShellBatchCollectionInput` ranges; `SourceAssemblyMaterialInput::input()`
returns borrowed TL material declaration ranges. These objects construct no
native mass, prepared shell reference, material history, node owner, clock,
constraint projection, contact or accepted trajectory. The next integration
step must give these declarations to TL's qualified startup and catalog APIs,
then bind native mass/J to the source connection groups. No caller-side mass
formula or readiness change occurs here.

Source copies share immutable ownership. Adapter copy/move construction is
supported; copied ranges rebind on every `input()` call, and material curves
retain shared source ownership. Adapter assignment is deliberately deleted so
allocation failure cannot mix provenance and old borrowed pointers. A moved-
from adapter may only be destroyed; source `data()` rejects a moved-from handle.

Owning targets are `robo_dyna_source_assembly`,
`robo_dyna_source_assembly_shell_input` and
`robo_dyna_source_assembly_material_input`. They share the same `output/ArtifactIO.cmake` owning target in production
and standalone builds; there is no separately defined reader copy of the
artifact library. The opt-in test target is
`robo_dyna_source_assembly_input_check` with 14 host test cases. The root option
is `ROBO_DYNA_ENABLE_SOURCE_ASSEMBLY_INPUT_CHECKS=ON`; standalone configuration:

```
cmake -S modelio/source_assembly -B NEW_BUILD_DIR \
  -DChrono_DIR=EXISTING_CHRONO_INSTALL_CMAKE_DIR \
  -DROBO_DYNA_TL_ROOT=ABSOLUTE_TL_CHECKOUT \
  -DROBO_DYNA_SOURCE_ASSEMBLY_INVENTORY=ABSOLUTE_FROZEN_INVENTORY
```

The 14 cases cover pinned counts/coordinate bits, all native input mappings,
curve/material retention, source group membership and release metadata,
copy/move lifetime, explicit rate policy, wrong hashes, duplicate JSON keys,
late source-ID/member/declaration errors, scope changes and each admission cap.
The owning standalone target passes all 14 host cases (2026-09-10). Workspace
evidence: `crash-work/reports/source-assembly-input-{configure,build,tests}-1`.
This reader gate does not advance native mechanics or use a GPU.
