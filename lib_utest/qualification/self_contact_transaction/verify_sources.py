#!/usr/bin/env python3
"""Focused source proof for the fixed self-contact transaction."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "lib_src/collision/SelfContactTransaction.h"
TYPES = ROOT / "lib_src/collision/SelfContactTransactionTypes.h"
CANDIDATE = ROOT / "lib_src/collision/self_contact_transaction/Candidate.cpp"
TRANSACTION = ROOT / "lib_src/collision/self_contact_transaction/Transaction.cpp"
STREAMING = ROOT / "lib_src/collision/self_contact_transaction/Streaming.cpp"
BROADPHASE = ROOT / "lib_src/collision/SelfContactBroadphase.cpp"
BROADPHASE_TYPES = ROOT / "lib_src/collision/SelfContactBroadphaseTypes.h"
CMAKE = ROOT / "lib_src/collision/SelfContactTransaction.cmake"
BAZEL = ROOT / "lib_src/collision/BUILD.bazel"
QUAL_CMAKE = Path(__file__).resolve().parent / "CMakeLists.txt"
QUAL_BAZEL = Path(__file__).resolve().parent / "BUILD.bazel"
CUDA = Path(__file__).resolve().parent / "CudaTest.cu"
MEDIUM = Path(__file__).resolve().parent / "MediumCouponTest.cpp"
OWNER = (Path(__file__).resolve().parent /
         "../physical_publication/OwnerStartup.cu").resolve()


def require(text: str, token: str, where: Path) -> None:
    if token not in text:
        raise RuntimeError(f"{where}: missing {token!r}")


header = HEADER.read_text()
types = TYPES.read_text()
candidate = CANDIDATE.read_text()
transaction = TRANSACTION.read_text()
layout_path = ROOT / "lib_src/collision/self_contact_transaction/Layout.cpp"
layout = layout_path.read_text()
for token in (
    "AuthenticateAssemblyView(token, view)",
    "AssemblyRangeDisjoint(",
    "physical_activity.CaptureAccepted(",
    "activity_receipt.activity()",
    "owner, token, view, activity,",
    "{state.buffers.accepted_events, event_count}",
    "candidate_source.Begin(",
    "candidate_source.Next(",
    "candidate_source.Finish(",
    "MergeAcceptedEventChunk(",
    "FinalizeAcceptedEventLedger(",
):
    require(transaction, token, TRANSACTION)
require(candidate, "state.force.Authenticates(assembly.force_)", CANDIDATE)
for token in (
    "physical_activity.CapturePrepared(",
    "physical_diagnostics, prepared",
    "assembly.activity_",
    "activity_receipt.activity()",
):
    require(candidate, token, CANDIDATE)
for token in (
    "CopyAcceptedRigidGroups(",
    "CopyPreparedRigidGroups(",
    "ConservativeSweptParentBounds",
    "ClassifyCandidatePairMotion(",
    "FilterAcceptedFacetPairs(",
    "DescribeMotionFailure(",
):
    require(candidate if token != "FilterAcceptedFacetPairs(" else
            transaction, token,
            CANDIDATE if token != "FilterAcceptedFacetPairs(" else
            TRANSACTION)
if BROADPHASE.exists() and BROADPHASE_TYPES.exists():
    for token in ("ConservativeSweptParentBounds",
                  "swept_parent_bounds", "staged_bounds"):
        require(BROADPHASE.read_text() + BROADPHASE_TYPES.read_text(),
                token, BROADPHASE)
for token in (
    "broadphase.forecast.retained_source_bytes",
    "force.forecast.retained_active_use_bytes",
    "shared_backing_discount_bytes",
):
    require(layout, token, layout_path)

storage_path = ROOT / "lib_src/collision/self_contact_transaction/Storage.h"
storage = storage_path.read_text()
for forbidden in ("has_rigid_motion",):
    for text in (
        candidate, transaction,
        (ROOT / "lib_src/collision/self_contact_transaction/Source.cpp").read_text(),
        storage,
    ):
        if forbidden in text:
            raise RuntimeError(
                f"transaction retains global rigid-motion rejection {forbidden!r}")
for token in ("SelfContactForceAssembly force",
              "SelfContactPhysicalActivity physical_activity",
              "SelfContactBroadphase broadphase",
              "FixedTriangleFeatureDiscovery accepted_discovery",
              "FixedTriangleFeatureDiscovery candidate_discovery",
              "StreamingCandidateSource candidate_source",
              "SelfContactCurrentRegularity regularity",
              "RepresentedIntervalCrossing crossing",
              "ShellPhysicalScratchParticipation participation"):
    require(storage, token, storage_path)
for token in ("accepted_rigid_groups", "prepared_rigid_groups",
              "node_rigid_groups", "parent_motion", "facet_motion",
              "chunk_crossings", "chunk_motion_actions",
              "swept_parent_bounds",
              "swept_facet_bounds"):
    require(storage, token, storage_path)

for token in (
    "roster_entry()",
    "AssembleAccepted",
    "SealCandidate",
    "DiscardTrial",
):
    require(header, token, HEADER)

for forbidden in (
    "SelfContactForceAssembly&",
    "ShellPhysicalScratchParticipation&",
    "participation()",
    "force()",
    "SelfContactActivityView",
    "SelfContactForceEventView",
    "SelfContactCandidateEvidence",
):
    if forbidden in header:
        raise RuntimeError(f"{HEADER}: exposes private transaction state {forbidden!r}")

for token in (
    "class SelfContactTransactionReceipt",
    "SelfContactPreparedActivityReceipt activity_",
    "ShellPhysicalScratchParticipationReceipt participation_",
    "scratch_receipts()",
    "class SelfContactAcceptedAssemblyReceipt",
    "SelfContactAcceptedActivityReceipt activity_",
    "AcceptedSymmetricVfEeRejectIntersectionV2",
    "motion_certified_linear_separated",
    "exact_crossing_pairs",
    "exact_crossing_work",
):
    require(types, token, TYPES)
for forbidden in (
    "RequireAllSelectedParentsActiveV1",
    "parent_activity_bytes",
):
    if forbidden in types:
        raise RuntimeError(f"{TYPES}: retains obsolete activity seam {forbidden!r}")

arena_path = ROOT / "lib_src/collision/self_contact_transaction/Arena.cpp"
arena = arena_path.read_text()
for forbidden in ("activity_base", "activity_current"):
    if forbidden in storage or forbidden in arena:
        raise RuntimeError(
            f"{storage_path}: retains duplicate transaction activity storage")

for token in (
    "CopyAccepted",
    "CopyPrepared",
    "state.regularity.Certify",
    "state.candidate_discovery.Discover",
    "state.crossing.Certify",
    "state.broadphase.Evaluate",
    "ValidateCandidatePublications",
    "FoldPolicyOutcomes(",
    "summary.exact_crossing_pairs += pair_count",
    "summary.exact_crossing_work = crossing_work",
    "participation.SealSelfContactCandidate",
):
    require(candidate, token, CANDIDATE)

for forbidden in (
    "accepted_facet_pairs",
    "candidate_facet_pairs",
    "accepted_broadphase_pairs",
    "candidate_broadphase_pairs",
    "buffers.represented_paths",
):
    if forbidden in transaction or forbidden in candidate:
        raise RuntimeError(
            f"transaction retains whole-batch range {forbidden!r}")

force_call = transaction.index("force.AssembleAccepted")
accepted_broadphase = transaction.index("broadphase.Evaluate")
accepted_events = transaction.index("BuildAcceptedEvents")
record_call = transaction.index(
    "participation.RecordSelfContactAcceptedAssembly")
if not accepted_broadphase < accepted_events < force_call < record_call:
    raise RuntimeError(f"{TRANSACTION}: participation recorded before force success")

for path in (
    CANDIDATE,
    TRANSACTION,
    STREAMING,
    ROOT / "lib_src/collision/self_contact_transaction/Source.cpp",
):
    text = path.read_text()
    for forbidden in ("std::vector", "std::map", "std::function",
                      "cudaMalloc", "new "):
        if forbidden in text:
            raise RuntimeError(
                f"{path}: forbidden per-attempt allocation token {forbidden!r}")

source = (
    ROOT / "lib_src/collision/self_contact_transaction/Source.cpp").read_text()
values = (
    ROOT / "lib_src/collision/self_contact_transaction/Values.cpp").read_text()
for token in (
    "FixedContactFacetReadCursor facet_reader",
    "facet_reader.Initialize(*facet_binding)",
    "facet_reader.Describe(value.surface_parent, local)",
    "buffers.facet_descriptors[global] = *described.facet",
):
    require(source, token, storage_path)
if "active_use.facets()->Describe(" in source:
    raise RuntimeError(
        f"{storage_path}: transaction startup retains checked per-facet Describe")
if source.index("facet_reader.Describe(") > source.index(
        "buffers.facet_descriptors[global] = *described.facet"):
    raise RuntimeError(
        f"{storage_path}: borrowed facet is not copied immediately after read")
for token in (
    "AcceptedEventCertificateKind::EdgeEdge",
    "SelfContactPairStatus::AdmittedEdgeEdge",
    "SameEdgeEdgeCertificateIdentity",
    "Complete accepted VF+EE event set",
    "HashEventIdentity",
    "CompareSelfContactForceEventIdentity",
    "SameSelfContactForceEventIdentity",
    "SameParentPair(certificate.discovery, crossing.key)",
    "SameVertexFaceOwners(",
):
    require(source + values, token, storage_path)
if "Hash(input[i].event.feature, &hash)" in values:
    raise RuntimeError(
        f"{storage_path}: accepted ledger hashes geometry without ownership")
if "CoveredByAdmittedVertexFace" in source:
    raise RuntimeError(
        f"{storage_path}: EE policy retains facet-broadened VF coverage")
for token in (
    "RepresentedByAcceptedEdgeEdge",
    "EE crossing lacks its exact accepted EE certificate",
):
    require(values, token, storage_path)

for wiring in (CMAKE, BAZEL):
    text = wiring.read_text()
    for token in ("self_contact_transaction/Arena.cpp",
                  "self_contact_transaction/Candidate.cpp",
                  "self_contact_transaction/Limits.cpp",
                  "self_contact_transaction/Source.cpp",
                  "self_contact_transaction/Streaming.cpp",
                  "self_contact_transaction/Transaction.cpp",
                  "self_contact_transaction/Values.cpp"):
        require(text, token, wiring)
    require(text, "self_contact_physical_activity", wiring)

for token in (
    "SELF_CONTACT_TRANSACTION_CUDA",
    "CudaTest.cu",
    "MediumCouponTest.cpp",
    "self_contact_transaction_medium_coupon",
    "tl_self_contact_transaction",
    "rigid-cin-response",
    'LABELS "unit;',
    'LABELS "coupon;',
):
    require(QUAL_CMAKE.read_text(), token, QUAL_CMAKE)
for token in ("host_check", "source_check", "root_cuda_sources"):
    require(QUAL_BAZEL.read_text(), token, QUAL_BAZEL)
require(QUAL_BAZEL.read_text(), '":root_cuda_sources"', QUAL_BAZEL)
require(QUAL_BAZEL.read_text(), "m2-rigid-cin-response", QUAL_BAZEL)
require(QUAL_CMAKE.read_text(), "symmetric-edge-area", QUAL_CMAKE)
require(QUAL_BAZEL.read_text(), "symmetric-edge-area", QUAL_BAZEL)
for token in (
    "DecisionCount = 262144",
    "ChunkCapacity = 257",
    "ExpectedPolicyDigest",
    "ClassifyCandidatePairMotion(",
    "MergeAcceptedEventChunk(",
    "FinalizeAcceptedEventLedger(",
    "summary.digest, ExpectedPolicyDigest",
    "sizeof(FixedStorage) < 512u * 1024u",
):
    require(MEDIUM.read_text(), token, MEDIUM)

cuda = CUDA.read_text()
for token in (
    "InitializeExecutionCatalog",
    "InitializeExecution",
    "execution.Initialize",
    "rig.InitializeAgainst(physical)",
    "surface.Initialize(\n        physical",
    "publication.ConfigurePhysicalScratchParticipation(\n"
    "                rig.owner, physical",
):
    require(cuda, token, CUDA)
if "rig.fixture.physical" in cuda:
    raise RuntimeError(
        f"{CUDA}: transaction fixture fell back to legacy physical authority")
for forbidden in ("SelfContactCandidateEvidence",
                  "SelfContactCrossingDecision",
                  "SelfContactForceEventView",
                  "fixture.activity"):
    if forbidden in cuda:
        raise RuntimeError(
            f"{CUDA}: caller still supplies authority {forbidden!r}")
for token in (
    "AcceptedInteriorEeForceCandidateRetryAndRollbackKeepForceSti",
    "ActualT3RemovalFiltersCandidateAndLongInactiveRetryCommits",
    "ActualMergedRigidBodyExcludesDiscoveredVfBeforeForceOrSti",
    "ActualMergedPartAndPlainBodiesUseMergedWrenchesBeforeInverseResponse",
    "ContactConstraintLayout::SameMergedParts",
    "ContactConstraintLayout::MergedPartAndPlain",
    "CopyPreparedForceStage",
    "endpoint_inverse_sum",
    "SelfContactTransactionStatus::UnsupportedMotion",
    "common, prepared, accepted",
    "removing_parents()",
    "skipped_parents()",
    "boundary_vertex_edge_event_count",
    "motion_certified_linear_separated",
    "exact_crossing_pairs",
    "exact_crossing_work",
):
    require(cuda, token, CUDA)

nodal_header = (ROOT / "lib_src/solvers/FENodalState.h").read_text()
nodal_source = (ROOT / "lib_src/solvers/NodalOwnerStream.cpp").read_text()
for token in ("BorrowOwnerStream", "ValidateOwnerStream"):
    require(nodal_header, token, ROOT / "lib_src/solvers/FENodalState.h")
    require(nodal_source, token, ROOT / "lib_src/solvers/NodalOwnerStream.cpp")
for path in (ROOT / "lib_src/solvers/CMakeLists.txt",
             ROOT / "lib_src/solvers/BUILD.bazel"):
    require(path.read_text(), "NodalOwnerStream.cpp", path)
force_initialize_path = (
    ROOT / "lib_src/collision/self_contact_force/Initialize.cpp")
force_initialize = force_initialize_path.read_text()
for token in ("BorrowOwnerStream", "ValidateOwnerStream",
              "cudaMemcpyAsync", "cudaStreamSynchronize"):
    require(force_initialize, token, force_initialize_path)

owner = OWNER.read_text()
for token in (
    "InitializeAgainst(const fe::ShellPhysicalBinding& physical",
    "InitializeMapped(q,physical",
    "InitializeMapped(t,physical",
    "InitializeMapped(b,physical",
    "InitializePhysical(owner,physical",
):
    require(owner, token, OWNER)

print("fixed self-contact transaction source proof: PASS")
