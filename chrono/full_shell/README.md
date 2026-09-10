# Full-shell binary frame presentation

`FullShellFrameGeometry` connects existing immutable source mapping and binary
frame records to Chrono's indexed mesh and established color policies. It is a
presentation adapter. It does not construct `ReplayInfo`, certify a complete run,
select native material models, publish accepted mechanics, add a solver/clock or
introduce another rendering format. There is no new CLI schema dispatch here.

## Inputs and ownership

Initialize once with `PreparedSourceMapping`, the matching full-shell `Context`,
and explicit `FrameGeometryOptions`. The adapter reuses the mapping's context
construction and point-layout identity to check every source/point association.
Opaque native-family IDs retain their caller-defined meaning; checking the mapping
is not a native material or runtime admission result.

The default geometry limits remain small. `ReplayGeometryLimits::Vehicle()` is an
explicit opt-in for the original no-tire 359785 nodes, 349645 parents and 677989
triangles. A conservative overflow-checked workspace budget precedes decoding or
allocation. It includes mesh/staging, source/color indices, parent fields and
transient context/index/array work. Already owned source/context/frame memory and
Chrono/VSG driver allocations belong to their separate guards. Default additional
workspace cap is 384 MiB; callers may lower it or explicitly select up to 512 MiB.

Immutable topology is decoded once from the existing `triangles` and
`triangle_parents` arrays. Triangle EID/PID comes from the original ordered parent
records. Unsigned64 source IDs are never narrowed or regenerated. No geometry
welding, transform, alternate shell diagonal or invented source binding is used.

`Update(record, expected_stamp)` requires the caller's independently authenticated
expected phase, checks all existing binary record fields, and copies coordinates
into staged `ChVector3d` storage without changing binary64 bits. The shared Chrono/
VSG representability check is the unchanged former scene geometry check, now owned
by `robo_dyna_replay_geometry_values`. Both binary64 and actual binary32 display
triangle collapse reject. It is display validation, not a mechanics geometry guard.

The adapter may seek any caller-expected record; it does not manufacture a timeline
or claim complete epoch coverage. The outer run reader must validate the complete
manifest/interval/index sequence. Geometry, parent fields and selected scalar
colors swap only after every check succeeds. A rejected update preserves the last
visible frame, colors and stamp. The live mesh handle keeps its identity and must
not be retained as an immutable historical snapshot or mutated behind the adapter.
No frame mesh or fields are exposed before the first successful update.

## Native, inapplicable and unavailable fields

`ParentPlasticMaxima` remains the single display reduction of the binary native
point array. All point values remain in the original record. Native points map to
`ReplayScalarApplicability::NativeValue`; `NotApplicable` and `Unavailable` remain
separate tags. Their legacy double storage slot has a canonical uninterpreted zero
marker. The renderer branches on applicability first and never plots that marker
as zero plastic strain. The code does not use `value_or(0)`.

The tags and parent IDs are fixed at initialization and checked on every update.
Native values must be finite and nonnegative. Non-native tags must contain the
canonical storage marker; unknown tags and changed applicability reject. Existing
`ReplayParentScalar{EID,value}` callers keep the default native interpretation.

Native fields require a caller-declared fixed positive scalar maximum in every
mode; no frame autoscaling is introduced. If there are no native values, the scale
must be zero and plastic-strain color mode rejects. Part-ID or uniform mode remains
available. Automatic mode in this new full-shell adapter resolves to part ID;
legacy `AcceptedReplayScene` automatic behavior remains unchanged.

| Applicability | Scalar display | Numeric interpretation |
| --- | --- | --- |
| Native value | Existing blue/yellow/red fixed ramp | Maximum stored native PLA |
| Not applicable | Gray `(0.48,0.50,0.52)` | None |
| Unavailable | Purple `(0.72,0.30,0.74)` | None |

The adapter exposes a typed scalar legend with all three counts and the scale.
The existing scene exposes the same legend. Existing overlay/capture metadata adds
missing-field labels/counts only when those tags occur; legacy capture bytes stay
unchanged. The narrow legacy scene still rejects a claim of native plasticity when
all fields are missing. Full-shell reader/scene admission remains a later contract.

Part mode uses the unchanged original-PID palette/version/seed and source legend.
Uniform and part modes still validate every native/missing field before publishing
geometry. Both display triangles of a Q4 parent use that same parent's field.

## Qualification and next integration

Eight author host tests pass under one CPU and a 512 MiB process address-space cap:
legacy/default scalar ramp, typed missing colors and immutable tags, no-native
scale handling, resource/renderer checks, full original no-tire binary frame
roundtrip in both modes, exact last-node/parent coverage, wrong context/phase/NaN
rollback and retry, and complete unavailable-only source handling.
The actual-source tests use an opaque formatting-only native family and two
synthetic three-point fields. Their translated frame and field markers test I/O
and presentation only; they are not a simulated crash or material validation.

The existing scene/metadata target additionally owns three new presentation-only
contracts for typed missing fields, failed publication and truthful legend metadata.
Shared scene/metadata sources passed bounded host syntax checks. Root owns the
larger legacy scene/actual archive regression and VSG build/render gate.

```sh
cmake -S chrono/full_shell -B <build> -DCMAKE_BUILD_TYPE=Release \
  -DChrono_DIR=<install>/lib/cmake/Chrono \
  -DROBO_DYNA_FRAME_GEOMETRY_CANONICAL_FIXTURE=<canonical-assets> \
  -DROBO_DYNA_FRAME_GEOMETRY_SCOPE_FIXTURE=<yaris-full-shell-scope-6.json>
cmake --build <build> --target robo_dyna_full_shell_frame_geometry_check --parallel 1
ctest --test-dir <build> --parallel 1 --output-on-failure
```

`robo_dyna_full_shell_frame_geometry` links existing source mapping and
`robo_dyna_replay_geometry_values`; no TL runtime, CUDA or VSG dependency is added.
`chrono/replay/CMakeLists.txt` owns the shared values and existing scene targets;
viewer and standalone geometry checks reuse it. The full accepted-run publisher,
authenticated full-shell replay sidecar/kind, placed-wall admission and CLI capture
remain the explicit next gates described in the workspace replay integration plan.
