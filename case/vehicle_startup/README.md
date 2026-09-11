# Original vehicle native shell references

`VehicleShellReferences::Prepare(VehicleSourcePlan, ReferenceLimits)` assesses
every retained original parent in canonical source order. The immutable result
retains its complete authenticated source plan, including unresolved declarations
and all original connection evidence. Copy/move construction shares that backing;
assignment is deleted. This is reference readiness, not runtime admission.

For a supported declaration the factory packs the actual source NIDs, original SI
coordinates, thickness, E, nu and density and calls TL's existing
`qeph::InitializeReference` or `t3::InitializeReference`. Those are also the
producers used by `ShellBatchBinding`. Only packing is shared with the component
adapter; there is no new geometry, mass, inertia or material formula. Original
warping and signed-zero coordinates are preserved in `ReferenceData::input`.
The native projected frame, area, local mass and physical/added/total J are kept
exactly as returned. In particular total J is not reconstructed from partitions.

Each original EID has one row with PID, MID, SID, original line, canonical parent
and part indices. Unresolved rows have no native family or reference. Attempted
native failures retain their exact family/status and no stale result. Evaluation
continues, so `first_error()` means the first native rejection in original parent
order, not the binding's Q-before-T error precedence. Source association/packing
or budget errors throw before an immutable result is published.

The historical plan-only scope has 349645 retained parents. Its declarations support attempting
259200 QEPH and 19101 T3 references; 71344 other parents remain unresolved. These
are input counts, not a claim that the native geometry accepts every attempt.
The source gate records actual success/rejection counts and first failing
family/EID/PID/line/status in XML. A passing coverage test can contain reported
native rejections: the report must be read before claiming reference readiness.

Root qualification on 2026-09-10 reports **278301 successes and zero rejections**,
with all 71344 unresolved rows retained. Three value and three complete-source
functions pass without skips in `vehicle-native-reference-root-functions-1/`.
The actual 149/631-shell V3 projections match every native reference field.
The accounted forecast is 463597300 bytes; the guarded test took 1.007 s with
478031872 bytes sampled process-tree RSS. These are original geometry/startup
results, not a vehicle trajectory or a complete mechanics admission.

There is no partial `ShellBatchBinding`, nodal/global mass sum, rigid model, owner,
accepted state, clock, force or history here. Local reference validity does not
resolve source failure laws, MAT_RIGID, unsupported sections/materials, auxiliary
coefficients, beam/tied load paths, or the selected vehicle boundary. This module
does not require unrelated nonshell assemblies to become runtime prerequisites.

The new `Prepare(VehicleSectionResolution, ReferenceLimits)` overload retains the
separate immutable resolution handle, accessible through `resolution()`. That
accessor is null for the historical overload. Both overloads use the same geometry
decode, source packing and native append path. Resolution output2 makes 326082
parents available for native assessment while retaining 23563 unresolved rows.
It adds 47781 ordinary positive-FAIL declarations with their original MID/SID,
thickness, coefficients and source blocks. Failure policy does not change the
reference geometry or native M/J producer. Actual success/rejection counts for
this extended scope require the new owning source gate; declaration resolution
alone is insufficient.

## Memory admission

`ForecastReferences` validates explicit count and byte limits before allocating
reference or decode buffers. The default cap is 524288 parents/nodes and 512 MiB
of accounted startup payload. The forecast adds:

- One `VehicleSourcePlan::startup_budget_bytes()` bound for retained shared
  source. It deliberately overcounts already-freed source parser/scratch memory.
- Exactly one complete row buffer and separate Q/T reference capacities bounded
  by `min(source family count, supported parent count)`. There is no per-parent
  allocation or maximum-size union. Capacity is reserved before the single parent
  traversal; failed attempts do not leave a reference object in active storage.
- The five typed canonical decode buffers and largest simultaneous temporary
  byte string used by the existing checked array codec.
- Fixed owning/vector control objects and a conservative 32 KiB startup stack
  allowance. Allocator bookkeeping and process RSS remain separate guard concerns.

The resolved overload selects the explicit `ReferenceLimits::ResolvedSections()`
profile, capped at 768 MiB. The legacy profile still rejects a byte cap above
512 MiB; the plan-only overload rejects the new profile. The resolution's source
bound already includes the historical plan, so shared backing is charged once.
For the frozen output2 fixture, before native-reference execution, the forecast is
540700156 bytes: source/resolution 283999415, reference capacity 180127936,
rows 22377280, decode 36956356 plus temporary 17205937, and fixed controls/stack
33232. Capacities remain conservative (326082 QEPH and 21301 T3 slots), not an
assertion of successful native family counts. The optional handle adds 24 control
bytes to the historical factory forecast; its references and counts are unchanged.

All products and additions are checked against the caller's cap before use.
The complete-source retry test intentionally retains two immutable assessments;
that test process therefore needs more than one assessment's individual budget.

## Owning host gates

Configure `case/vehicle_startup` with explicit `ROBO_DYNA_TL_ROOT`, `Chrono_DIR`,
`ROBO_DYNA_FULL_SHELL_RECORD_TESTS=OFF`, `ROBO_DYNA_SOURCE_MAPPING_TESTS=OFF`.
`robo_dyna_vehicle_reference_value_check` / CTest `vehicle_reference_values`
checks exact native results, signed zero, Q/T source-ID width, ordered late native
rejection with no stale packet, and packing failure/unchanged output/exact retry.

Enable `ROBO_DYNA_VEHICLE_REFERENCE_SOURCE_TESTS=ON` and supply the same explicit
`ROBO_DYNA_VEHICLE_{CANONICAL,SCOPE,DECLARATIONS,ELASTIC,MIXED}` paths as the source
plan gate. Build `robo_dyna_vehicle_reference_source_check` and run CTest
`vehicle_reference_source`. It checks all 349645 source rows, successful input
bits/local native M/J, actual native failure census, complete 149/631-parent V3
projections against existing binding inputs and every named native reference
field, immutable lifetime, cap rejection and exact whole-assessment retry.

The two targets are host-only. No CUDA runtime, driver, solver ownership or
native Fortran oracle is linked; donor arithmetic remains the already-qualified
TL QEPH/T3 producer. Full-source execution belongs to the serialized parent gate.

Enable `ROBO_DYNA_VEHICLE_REFERENCE_RESOLVED_TESTS=ON` and provide the preceding
source paths plus `ROBO_DYNA_VEHICLE_RESOLUTION` pointing to the authenticated
`yaris-vehicle-section-resolution-2.json`. Target
`robo_dyna_vehicle_reference_resolved_check` / CTest `vehicle_reference_resolved`
assesses every source row, verifies original material/thickness and donor M/J,
compares every previously supported reference and both V3 fixtures exactly, and
checks explicit profile admission, late budget failure and clean retry. This
qualification intentionally retains legacy and resolved assessments concurrently;
its process guard must cover that total rather than one factory's forecast.
