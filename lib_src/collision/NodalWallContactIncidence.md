# Ordered native-share incidence

`NodalWallContactModel.cpp` builds one immutable row for each compact contact
node during the existing admission/rate traversal. A row contains encoded
`4 * parent + local` slots in canonical parent/local order. Q4 has four slots;
T3 has three, with no padding incidence. Startup checks complete coverage before
publishing the prepared model.

`NodalWallContactArena` owns and rebases `node_count + 1` offsets and
`4 * parent_count` slot capacity inside its existing bounded arena. This adds
18,764 bytes of arrays for 1,030 nodes and 915 parents, plus header/alignment
accounting. It does not add a device allocation or enlarge a configured cap.

`NodalWallContactKernels.cuh` reads each row instead of scanning all parents for
each node. The point law, finite-wall query, arithmetic order, failure ordering,
parent reduction, force scatter and publication remain the existing operations.
Per-node source traversal changes from O(nodes * parents) searches to O(native
incidences). This is an indexing change, not a new contact approximation.

Qualification: `wall-incidence-build-1.json` and `wall-incidence-tests-1.json`
under the workspace reports record 83 passing functions in 13 affected groups.
Host checks cover exact shared/sparse-node incidence, permutation invariance,
all 3,549 actual-sized mixed-family slots, rebased pointers and capacity failures.
CUDA checks retain last-node/parent rejection, rollback and stable allocations.
Actual assembly archive parity and measured runtime remain the performance gate.
