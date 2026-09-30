# Original physical connectivity

`VehicleConnectivity::Prepare` consumes the immutable
`VehiclePhysicalAttachments`. The explicit `PrepareWithJoints` overload also
retains the prepared `VehicleJointModel` from that exact physical model. It borrows the exact prepared source/domain
through a retained shared handle and publishes one complete graph value after
all source associations and ordered supports pass. It does not parse source
cards, duplicate geometry, change coefficients, or advance an owner.

Two partitions use the minimum original NID as their deterministic label:

- Element incidence joins constitutive QEPH/T3/QBAT, solid18/24/6z, LAW44/LAW90 solid18, structural beam18 and TYPE13 /
  TYPE25 mechanical supports. Every repeated solid source slot stays present.
- Potential transfer adds the actual retained PART roots, plain groups and
  complete CIN secondary plus four ordered master slots. `PrepareWithJoints`
  adds the actual joint N1/N2 endpoints. A repeated native T3 slot remains
  present in the relation record.

Original rigid skins retain their EID/PID, source row and authenticated PART
root, without becoming constitutive edges. Each skin slot is checked against
that root's actual members. Point-mass cards are independent coefficient
attributions. They retain their own identity namespace and add no edges.
TYPE13/beam18 orientation N3, joint axis N3, equal coordinates, contact and
proximity add no edges. The legacy overload adds no joint edges. The fixed TYPE13 PID is the already authenticated source-reader
profile in `modelio/type13/ReadProperty.cpp` and `ReadGeometry.cpp`.

This first report preserves every selected typed relation and both complete
node-label arrays. A relation's source row indexes the retained typed source;
CIN's row reaches its exact NSV/master-rank receipt, and restricted rigid
groups reach their original/effective member and NSID evidence. JSON contains
all original NIDs, EIDs/PIDs, roles, root indices and ordered domain slots, so a
PID may be associated with several components without collapsing its rows.

Joint rows have separate spherical, revolute and cylindrical relation kinds.
They preserve native released-DOF masks 56, 8, 9 respectively, with bits ordered
Tx,Ty,Tz,Rx,Ry,Rz in the current joint frame. They do not lock those axes or
imply world-axis constraints. The retained typed source keeps the original
property, axis geometry and actual rigid-body association. Both complete rigid
bodies remain separate existing relations. The admitted model profile requires
zero optional stiffness/viscosity on released axes.

The partition makes CIN/joint support undirected only to compute weak potential
transfer. Its ordered CIN row still means secondary followed by four masters;
it does not prove reverse kinematic influence or constrained-DOF rank. There
is no current activity, force, stiffness, contact or solver clock in this value.

`required_joints()` is Pending without a joint handle and PreparedSourceOperators
with one. This refers to that source policy's selected operators; any original
boundary rows remain explicit in the V2 report. V5 has 44 operators and 0 boundary
rows. Omitted assemblies/auxiliary source policy and current CIN activity remain
Pending in both paths. The older source audit is in
`planning/YARIS_CONNECTIVITY_GATE.md`; remaining omission dispositions are in
`planning/YARIS_REMAINING_SHELL_CONNECTIONS.md`. This gate changes none of them.

## Memory and publication

`Preflight` checks the complete typed authorities and count limits before
graph allocation. Default hard limits are 524,288 nodes and relations,
4,194,304 ordered slots, 64 MiB additional phase storage, a 32 MiB report and
an inclusive 8 GiB source/graph reservation. The existing physical attachment
source bound is retained once, including its conservative source startup
phases; no inferred subtraction of equal-size backing occurs. With joints,
preflight first proves the exact shared physical/domain and rigid group/member
backing, then adds the existing `additional_owned_payload_bytes()` result.
The joint input is already prepared; this value neither rebuilds it nor charges
a duplicate complete physical model. Preparation stages the optional immutable
handle and graph together; failure never publishes a partially populated graph.

Owned graph accounting includes the shared handle/control reserve, storage
objects, a bounded stack reserve, two u64 label arrays, node role bytes, 32-byte
relation rows and u32 ordered slots. Union-find scratch is one u32 per node.
Report construction follows scratch retirement, so additional storage is the
owned graph plus the larger of scratch and report reservation. These are
payload/reservation values, not allocator RSS or driver estimates. Copies of
the handle share the same backing; a caller retaining an independently
prepared second graph must reserve that second graph separately.

The renderer first counts the entire report against the cap without allocating
a text buffer, then writes into one reserved string. It never truncates rows.
The caller publishes through existing `ArtifactIO`; the original test uses a
fresh create-only path. Failed preparation or report construction leaves an
existing immutable value and file untouched. Actual source execution must
measure the complete JSON under the unchanged 32 MiB cap.

## Qualification

The legacy tiny owning target retains four tests: an independent explicit-adjacency BFS
oracle under reversed relation order; nonedges and repeated CIN slots;
last-slot/type failure with unchanged output and retry; and complete JSON with
exact-cap / cap-minus-one checks. The original target adds two functions for
all typed source counts, every support and CIN slot, source identity,
component counts and report publication, plus complete-budget rejection,
retry and shared source lifetime.

Expected original extents are 372,435 nodes and 371,413 relation rows:
349,645 shells (5,102 rigid skins), 2,412 solids, 4,442 TYPE13, 2,828 TYPE25,
20 PART roots, 753 plain groups, 11,165 CIN and 148 literal mass cards. The
component counts are measured outputs; there is no guessed one-component
assertion. The existing original attachment fixture needs CUDA to reproduce
its qualified source search. The graph itself is host-only; original/GPU
execution belongs to the root qualification lane.

Author evidence: `vehicle-connectivity-author-1` passes all four tiny functions
in 4.261 s, sampled peak 323,117,056 B; `vehicle-connectivity-syntax-1` compiles
the complete production and original test units in 8.017 s, sampled peak
390,578,176 B. Both use one CPU / 512 MiB; neither runs original-source or GPU
work. A final warning-clean syntax pass is recorded separately at freeze.


## V5 report and gate

Original relation codes 0..11 and 32-byte relation storage are preserved. New
kinds append after them. V1 inventories retain their original JSON schema,
kind table, relation shape and semantics. Any extended family or supplied joint
model selects `robo_dyna.original_physical_connectivity.v2`, with the complete
kind table, explicit released masks, joint model identity and omitted boundary
count. Every node label and ordered relation remains serialized; no component
label is guessed or truncated.

The four new tiny functions exercise the production typed-input/row helpers:
LAW44 repeated 8 slots, LAW90 eight slots, mechanical beam N1/N2 only, all three
joint kinds and native releases, axis-node exclusion, late source/endpoint/free-K
rejection and retry, and V2 exact report caps. Removing the only typed beam
bridge splits the tiny graph. Existing BFS, directed-CIN slot, skin, point-mass,
namespace and late-failure tests remain unchanged in meaning.

`ROBO_DYNA_CONNECTIVITY_SUPPORTS=ON` adds the root-only V5 target. It reuses
`physical_model/tests/supports/Source.cpp` and the existing `PreparePost`
classification factory, without creating a solver owner or advancing time.
Expected inventory: 376,930 nodes,374,179 relations,349,645 shells,4,980 solids,
142 beam18,4,442 TYPE13,2,828 TYPE25,779 rigid groups,11,165 CIN,154 mass cards,
and44 joints. Every extended solid/beam/joint slot is compared to its actual
retained typed/source row, including 113 repeated LAW44 cells.

The reachability test computes the dominant transfer component from actual
shell relations and requires every retained shell parent to be in it. It then
finds historical NIDs 2114321,2118226,2159133,2159217,2159443,2163548,2163575 in
the actual domain and compares their computed labels. Those NIDs identify the
old regions; they are not injected graph edges or expected component labels.
The test records observed counts/labels and writes the complete report even
when a component-count expectation fails. Full-source execution is still a
root qualification obligation; authored expectations are not a passing claim.

Author evidence: 8 tiny functions pass in `vehicle-v5-connectivity-author-1`
(7.19s,309,016KiB maximum child RSS). All 8 production and 4 original-fixture
syntax units pass in `vehicle-v5-connectivity-syntax-2` (11.64s,396,308KiB).
Both use one CPU / 512 MiB. No native/GPU/full-source execution was performed.
Root configure/build/test recipe is in `V5_HANDOFF.md`.
