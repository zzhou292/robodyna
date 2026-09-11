# Typed additive SI coefficients

`NodalCoefficientLedger` owns immutable startup coefficients from one complete
`ShellNodeMap` and optional qualified TYPE25 and TYPE13 producers. It adds no
owner, inverse M/J, DOF assignment, clock, contributor registration or full-source
completion authority. The original Initialize entry remains closed to those
producers. InitializeWithElementMass adds an optional immutable
ElementMassContributions through the same Impl and node arena. InitializeWithSolids
extends that same ledger with typed solid18/24/6z coefficients; later raw
rigid/CIN transformations remain separate.

The fixed policy `PreparedSI_Q_T_B_Type25_Type13_V1` visits QEPH, T3, QBAT in
retained parent/local-node order, then TYPE25 and TYPE13 in model/endpoint order.
Every value comes from an existing qualified native reference or endpoint
record. Each node's authoritative mass/native total J receives those terms
directly. Diagnostic partitions never reconstruct the authoritative total.
Global authoritative and diagnostic totals subsequently sum the nodal values in
declared domain-node order. The two reductions are distinct rounding stages.

The named extended entry reports
`PreparedSI_Q_T_B_Type25_Type13_ElementMass_V2`, including when its optional mass
producer is absent. It appends each point-mass record after TYPE13, preserving
source-row order. `order()` is now a per-instance query. The producer retains
original EID/NID/index/source-mass bits, a mass-to-kg scale and the exact domain;
it converts each mass once and never accepts scalar J. Nonnegative additive
TYPE5 is the only policy, with positive-conversion underflow/overflow rejected.
Zero source mass retains its occurrence without declaring a positive DOF.
The ledger leaves scalar J untouched and records the point-mass partition
separately. Part/set distribution and target/replacement mass are not admitted.

Shell thickness/placement and area-added inertia retain their original channels.
TYPE13 native total J already contains its numerical floor; `added_inertia` is
attribution, never an additional term. No physical TYPE13 J is computed by
subtracting two rounded SI channels. TYPE25 property M/J stays separately named.
No coefficient floor, conversion, per-element value or source identity changes.

This order differs from native global family/converted-spring-ID order and from
native working-unit sums followed by conversion. It intentionally does not
reconstruct generated native TYPE25 IDs. The qualification tests compare the
supplied SI terms with independent binary128 sums and conservative binary64
addition bounds, including an absolute gradual-underflow allowance. They check
both node sums and the later sum of rounded nodes, and the combined bound from
original terms. Positive terms that round away are permitted within those
bounds; wrong counts or source associations are never numerical tolerances.

The shell map retains exact local-to-domain NIDs and coordinate bits. TYPE13
retains its already checked complete domain, including the N3 distinction.
TYPE25 endpoint identity, property, domain index and coordinates are checked
before use. Q/T/B, TYPE13 and all three solid families share the original
structural EID namespace. ELEMENT_MASS nodal-mass IDs and TYPE25 original
spotweld WIDs have their own independent namespaces. Each typed producer keeps
its positive and unique IDs; equal numbers across namespaces do not merge or
drop physical records. Zero structural IDs and repeated
structural IDs reject in original validation order. Legacy pair bindings with
zero parent IDs remain supported by their unchanged original APIs, not this
new source-identified ledger. No reduced hash substitutes for complete identity.

This corrects an earlier overly broad mass/structural collision check. The
original Yaris has36 mass IDs also used by different TYPE13 beams;35 such mass
records belong to the retained source domain. For example, BEAM2409343 connects
3467984–3467985, while MASS2409343 adds mass at shell node2416473. The pinned
OpenRadioss mass converter emits a separate ADMAS type5 node/mass list. Source
identity and contribution order are retained; no material or mass arithmetic
changes. This typed nodal-mass distinction is not a generic keyword-ID policy.

Each row exposes exact Q/T/B/TYPE25/TYPE13/ELEMENT_MASS occurrence counts. Uncovered nodes
remain zero with no occurrences. This proves only that no admitted producer
touched the node; it assigns no absent, dependent or prescribed role. Even all
nodes covered cannot prove all original contributions are present. Later source
closure and DOF admission must establish the actual complete participant and
constraint inventory before an owner consumes these values.

Initialization validates count and complete payload limits before reading
borrowed records. It retains the entire map/model/adapter backing, deduplicating
shared domains only on actual backing identity, not semantic equality. Embedded
handles count once. Startup budget conservatively reserves the retained result
and the transient structural-ID index together (the index is actually released
before node allocation); both must fit the same cap. Node rows use one checked
HostArena. Failed identity, late arithmetic, cap or allocation leaves the handle
empty and retryable. Published copies share immutable lifetime-safe backing.

Small author qualification: seven host functions passed under one CPU/512 MiB.
They include all mixed-family partitions, legacy identity-map nodal bit parity,
TYPE13 floor-once attribution, namespace separation, late source/position errors,
retry, exact complete startup caps, shared versus equal-independent domains,
signed-zero coordinates, binary128 rounding checks and late global overflow.
The original-count synthetic fixture compiles but is root-run only: 349,645
parents / 359,785 shell nodes plus two explicit uncovered nodes. It is capacity
evidence, not an original-source model. The native control independently calls
the existing TYPE13 and TYPE25 coefficient donors at shared endpoints; its C++
packet compiles, while Fortran/native execution is root-owned. No new donor or
production/reference dependency is introduced. Strict legacy binding and mass
APIs are unchanged, and no GPU/runtime performance claim is made.

Root qualification (2026-09-11,50ec767): all9 host/native/full-count functions
and both unchanged native-source identities PASS in `nodal-coefficients-root-tests-1`.
The full-count ledger retains366695696 B with372290120 B complete startup.
All three owning Bazel targets PASS in `nodal-coefficients-owning-bazel-build-1`.
No production correction was needed. This is still the V1 additive scope above;
solid, point-mass, rigid/constraint stages and full-owner admission are separate.

The subsequent point-mass increment retains that V1 qualification as historical
evidence; private/node sizes grow and the complete forecasts are recalculated.
Author checks pass 11 host functions under one CPU/512 MiB (seven existing plus
four focused mass functions), including both-stage binary128 bounds. The new
native control reuses the unchanged authenticated HM_READ_ADMAS TYPE5 packet in
nodal_rigid_group/assembly; Fortran/native and the expanded full-count fixture
are root-run gates. No donor enters production. Original source-card selection
and completeness are app obligations, not implied by a positive nodal row.

## Selected solid coefficient extension

`SolidNodeContributions` accepts only prepared solid18, HEPH24 and S6Z reference
objects. It snapshots their exact source-slot native masses, EID/PID/SID/MID and
NID associations into the immutable physical domain, checking represented SI
coordinate bits. It reuses `NodalNodeDomain::Find`, `SourceIdentityIndex` and
`HostArena`. No density-times-volume reconstruction, invented scalar inertia,
second nodal ledger or mechanical state lives in this producer. Borrowed reference
objects may expire after initialization. Its `Matches` means coefficient identity;
force models still own complete reference geometry and material/profile identity.

The V3 ledger appends solid18, solid24 and S6Z after the prior V2 sequence, keeping
each family in supplied parent/source-slot order. Family mass and occurrence
partitions are diagnostics; authoritative nodal mass receives each source-slot
term directly. Solid-only nodes have positive mass and exactly zero scalar J.
That is coefficient evidence, not permission to divide by zero or manufacture a
rotational constraint. DOF admission is the next owner integration requirement.

Root numerical/source gate `solid-coefficients-root-tests-3` passes19 functions
(15 host,2 existing native,1 full-count,1 original-solid native) plus9 identities.
All908 adhesive/1309 brick/195 mapped wedge cells map onto5980 actual original
solid nodes and agree with independent native startup mass. The original-count
synthetic full shell ledger plus point masses now retains389722416 B and reserves
395316872 B at startup. This is capacity evidence, not the complete original
vehicle coefficient closure. Follow-up owning/affected gates are recorded in the
workspace execution status.

The expanded test channel array initially retained the old12-channel binary128
workspace and failed in `solid-coefficients-root-tests-2`. The oracle now derives
all work arrays from one shared15-channel count, and includes an additional
mixed-solid two-stage rounding control. No production coefficient equation or
numerical allowance changed. The initial passing source/native evidence and
failed test-harness attempt remain preserved.
