# Original physical connectivity

`VehicleConnectivity::Prepare` consumes the immutable
`VehiclePhysicalAttachments`. It borrows the exact prepared source/domain
through a retained shared handle and publishes one complete graph value after
all source associations and ordered supports pass. It does not parse source
cards, duplicate geometry, change coefficients, or advance an owner.

Two partitions use the minimum original NID as their deterministic label:

- Element incidence joins constitutive QEPH/T3/QBAT, solid18/24/6z and TYPE13 /
  TYPE25 mechanical supports.
- Potential transfer adds the actual retained PART roots, plain groups and
  complete CIN secondary plus four ordered master slots. A repeated native T3
  slot remains present in the relation record.

Original rigid skins retain their EID/PID, source row and authenticated PART
root, without becoming constitutive edges. Each skin slot is checked against
that root's actual members. Point-mass cards are independent coefficient
attributions. They retain their own identity namespace and add no edges.
TYPE13 N3, equal coordinates, contact, proximity and unimplemented joints
add no edges. The fixed TYPE13 PID is the already authenticated source-reader
profile in `modelio/type13/ReadProperty.cpp` and `ReadGeometry.cpp`.

This first report preserves every selected typed relation and both complete
node-label arrays. A relation's source row indexes the retained typed source;
CIN's row reaches its exact NSV/master-rank receipt, and restricted rigid
groups reach their original/effective member and NSID evidence. JSON contains
all original NIDs, EIDs/PIDs, roles, root indices and ordered domain slots, so a
PID may be associated with several components without collapsing its rows.

Required original joints, omitted assemblies and auxiliary source policies,
and current CIN activity/release are explicitly Pending. In particular the
38 required retained joints have no admitted operator here. Their detailed
source census and the 26 omitted non-shell PIDs are separate obligations in
`planning/YARIS_CONNECTIVITY_GATE.md`. A weak component is neither a DOF-rank
test nor a claim of complete load-path closure or current force transmission.

## Memory and publication

`Preflight` checks the complete typed authorities and count limits before
graph allocation. Default hard limits are 524,288 nodes and relations,
4,194,304 ordered slots, 64 MiB additional phase storage, a 32 MiB report and
an inclusive 8 GiB source/graph reservation. The existing physical attachment
source bound is retained once, including its conservative source startup
phases; no inferred subtraction of equal-size backing occurs.

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

The tiny owning target has four tests: an independent explicit-adjacency BFS
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
