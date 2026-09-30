# Ordinary-shell fixed-main startup

This is a host startup numerical producer for an explicitly ordered ordinary
exterior Q4/T3 surface. It owns no physical state, material, coefficient, gap,
clock or native runtime. The physical source factory retains all source/order,
fixed-DOF and lifetime authority. Moving-main runtime activation, erosion, coating,
internal faces, nonmanifold edges, disconnected vertex fans and nonorientable
surfaces are outside the first admitted profile.

Input primary order is preserved. The app source binding is responsible for
native deck/surface ordering. Node source IDs are arbitrary identities; node
indices select the actual input domain, and equal positions never weld nodes.
The original SH2SURF25 ordinary branch appends exact opposite-side permutations.
Both expanded-to-primary and primary-to-partner maps are explicit, so physical
material/thickness metadata is never inferred from generated IDs.

Sorted edge scratch only accelerates lookup. Admitted edges have one or two
primary faces, hence each side has at most one reversed native neighbor other
than its own partner. Duplicate faces and multiple shared edges reject. Native
I25NEIGH's direct-parent updates, closure and first-reference numbering remain
in source order. PREPARE_SPLIT_I25 supplies the normal-to-main CSR: ascending
expanded main/corner order, omitting the repeated T3 fourth slot.

Two stages remain distinct:

- `BuildStarter` reproduces the boolean Starter LBOUND, REAL4 primary normals,
  free-edge directions, boundary insertion order and neighbor average. Primary
  positions are converted to float before subtraction; free edges subtract
  MYREAL8 positions before float conversion. T3 slot3 stays initialized positive
  zero, including on the reversed side.
- `BuildFixedMain` consumes the immutable topology and an actual resolved,
  finite, strictly positive main-coefficient span. It calculates the selected
  all-active fixed-main NORMP field stage, including MAIN_NORM's prior bisector
  clear and FREE_BOUND insertion slots/counts. This is not dynamic TAGNOD/ACTNOR
  maintenance. The caller must establish fixed geometry and non-eroding activity.
  Starter and ready boundary values are never conflated.

Both stages retain the native float operation order and distinguish their floor
initializers. No normal renormalization, epsilon repair or captured output table
is substituted. Source-defined zero normals remain values, not certificates of
physical geometry admissibility. Nonfinite output rejects before publication.
The audited host RN/gradual-underflow/masked-trap environment is required.

`Preflight` uses the existing BoundedArenaLayout and reports aligned output and
private scratch payloads. Callers allocate the existing HostArena objects before
execution. No builder allocates or grows storage. Sorted-edge storage is reused
only after its lifetime ends, as neighbor-normal scratch. Every failed admission
or calculation preserves caller output bytes and view descriptors; publishing
starts only after all numerical/topology checks succeed.

Independent pinned native source and owning qualification belong under
`lib_utest/qualification/radioss_type25_fixed_main_startup`. Captured scene arrays
are expected test evidence only. Source authoring is not a compiler/runtime pass
or a full-vehicle contact/performance qualification.

The ready-stage snapshot is an immutable borrowed result of `BuildStarter` for
that source and generation. Structural admission checks spans, maps, reciprocal
neighbors and shared reference endpoints; it is not authentication of arbitrary
caller-generated or reordered CSR contents. The source owner retains the actual
producer result and its lifetime. No unchecked caller flag upgrades this view
into physical source authority.

`OrdinaryExteriorMovingMain` explicitly admits the same ordinary topology and
Starter numerical stage for the moving source producer. It does not admit the
all-active `BuildFixedMain` stage: that function rejects the moving profile
before touching output. The current-normal owner must perform native activation
and staged updates from the genuine Starter cache.


The default ManifoldTwoSided topology policy retains the original matcher and
forecast. Explicit NativeOrdinaryShell preserves native source-order multi-edge
neighbor selection and split normal references; Input-based Preflight includes
its additional scratch. Snapshot retains both motion profile and topology policy.
IRR11 is a source-bound warning with neighbor zero, never deletion of a primary.
General fixed-ready is not admitted: general moving sources start with genuine
Starter cache and use the separately qualified current NORMP stages. This new
policy requires its owning native/current-normal gates before selection; no full
Yaris topology or physical startup is implied by the bounded source cases.


## Explicit resolved coating roles

`Profile::ResolvedShellSides` requires `TopologyPolicy::NativeResolvedShellSides`.
Only this pair admits the trailing `PrimaryFace::side_role` values Ordinary,
CoatingForward and CoatingReversed. The input is an already-resolved ordered
shell roster; this producer does not classify solid membership or reproduce
upstream surface sorting. Raw native roles are 3/7, 4/8 and -4/-8 by Q4/T3 layout.
Negative coating reverses the supplied primary before appending its opposite.
Encoded coating partners carry the native G offset; the published partner map
still contains the decoded local main ID. Native neighborhood group separation,
reference-root ordering and REAL4 arithmetic are unchanged.

This profile copies every supplied role into `Snapshot::primary_roles` with an
exact P-element extent. Thus Forward versus Reversed provenance survives even
when different source connectivity normalizes to identical final geometry. The
source owner must retain genuine producer output; the table alone does not
authenticate a caller's source declaration. Both output and staging forecasts
include the sizeof-based table and alignment. Legacy forecasts and null/count0
role tables are unchanged. Generalized fixed-ready and runtime source admission
remain closed for this new profile. Current-normal value evaluation has its own
explicit resolved-shell profile and owning native/CUDA gates.

## Explicit mixed side stage

MixedSurface/NativeMixedSurface inputs use PreflightMixedSides and
BuildMixedSides, with true G=P+shell_count. Solid partner0 is explicit absence;
only shell/coating primaries append an opposite. The result is a distinct
MixedSidesSnapshot with no normal/readiness view.

Mixed inputs carry every raw source identity and raw-to-primary occurrence map.
SingleSourceFace keeps its original parent/localface. MultipleOrigins requires
count>=2 and unavailable parent/localface/source_id0. The primary ordinal and
global main ID identify the contact face, never a fabricated original EID.
These fields are source provenance, not later mechanical support/removal owners.

BuildStarter/Preflight(Input) reject the mixed profile until genuine
post-I25GAPM IELEM_M internal-support and final erosion disposition are provided.
Selected-clause exterior geometry does not justify an all-zero support mask.
Existing ordinary, general and resolved-coating normal/neighbor equations and
legacy arena region order remain unchanged.
