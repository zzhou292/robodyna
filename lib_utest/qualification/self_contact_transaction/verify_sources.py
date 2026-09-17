#!/usr/bin/env python3
"""Focused source proof for the fixed self-contact transaction."""
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[3]
HEADER = ROOT / "lib_src/collision/SelfContactTransaction.h"
TYPES = ROOT / "lib_src/collision/SelfContactTransactionTypes.h"
CANDIDATE = ROOT / "lib_src/collision/self_contact_transaction/Candidate.cpp"
RIGID_SWEEP = ROOT / "lib_src/collision/self_contact_transaction/RigidSweep.cpp"
ARENA = ROOT / "lib_src/collision/self_contact_transaction/Arena.cpp"
TRANSACTION = ROOT / "lib_src/collision/self_contact_transaction/Transaction.cpp"
STREAMING = ROOT / "lib_src/collision/self_contact_transaction/Streaming.cpp"
TASK_MASK = ROOT / "lib_src/collision/self_contact_transaction/TaskMask.cpp"
BROADPHASE = ROOT / "lib_src/collision/SelfContactBroadphase.cpp"
BROADPHASE_TYPES = ROOT / "lib_src/collision/SelfContactBroadphaseTypes.h"
CMAKE = ROOT / "lib_src/collision/SelfContactTransaction.cmake"
BAZEL = ROOT / "lib_src/collision/BUILD.bazel"
FILTER_HEADER = ROOT / "lib_src/collision/SelfContactFilterCertificates.h"
FILTER_SOURCE = ROOT / "lib_src/collision/SelfContactFilterCertificates.cpp"
QUAL_CMAKE = Path(__file__).resolve().parent / "CMakeLists.txt"
QUAL_BAZEL = Path(__file__).resolve().parent / "BUILD.bazel"
CUDA = Path(__file__).resolve().parent / "CudaTest.cu"
MEDIUM = Path(__file__).resolve().parent / "MediumCouponTest.cpp"
ACCEPTED_TEST = Path(__file__).resolve().parent / "AcceptedEventTest.cpp"
VALUE_TEST = Path(__file__).resolve().parent / "ValueTest.cpp"
RIGID_SWEEP_TEST = (
    Path(__file__).resolve().parent / "RigidSweepBoundsTest.cpp")
OWNER = (Path(__file__).resolve().parent /
         "../physical_publication/OwnerStartup.cu").resolve()


def require(text: str, token: str, where: Path) -> None:
    if token not in text:
        raise RuntimeError(f"{where}: missing {token!r}")


header = HEADER.read_text()
types = TYPES.read_text()
candidate = CANDIDATE.read_text()
rigid_sweep = RIGID_SWEEP.read_text()
arena = ARENA.read_text()
for token in (
    "max_nonlinear_subdivision_work_per_pair",
    "max_nonlinear_subdivision_work_per_chunk",
    "max_stream_nonlinear_subdivision_work",
    "max_nonlinear_subdivision_depth",
    "motion_certified_nonlinear_separated",
    "motion_certified_nonlinear_accepted_coverage",
    "motion_certified_nonlinear_exact_exclusion",
):
    require(types, token, TYPES)
transaction = TRANSACTION.read_text()
task_mask = TASK_MASK.read_text()
require(task_mask, "BuildFixedTriangleFeatureTaskMask(", TASK_MASK)
if "VertexInFacet(" in task_mask or "EdgesShareEndpoint(" in task_mask:
    raise RuntimeError(
        f"{TASK_MASK}: chunk mask construction duplicates shared topology logic")
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
    "MergeAcceptedEventIdentityChunk(",
    "CanonicalizeAcceptedEventIdentityCensus(",
    "VerifyAcceptedEventIdentityChunk(",
    "MergeAcceptedEventChunk(",
    "FinalizeAcceptedEventLedger(",
    "BuildLocalFeatureTaskMasks(",
    "accepted_discovery.DiscoverMasked(",
    "const bool direct_ledger",
    "? state.storage_forecast.event_hash_capacity",
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
    "BuildRigidMemberSweepBounds(",
    "BuildRigidFacetQuadraticCoefficients(",
    "CertifyQuadraticFacetSeparation(",
    "nonlinear_subdivision_work_per_pair",
    "nonlinear_subdivision_work_exhausted",
    "offending_quadratic_lower",
    "offending_swept_bounds",
):
    require(candidate if token != "FilterAcceptedFacetPairs(" else
            transaction, token,
            CANDIDATE if token != "FilterAcceptedFacetPairs(" else
            TRANSACTION)
require(candidate, "chunk_nonlinear_results[raw_pair].work;", CANDIDATE)
if "Rigid-arc separator did not replace one exact unsupported crossing" in candidate:
    raise RuntimeError(
        f"{CANDIDATE}: certified nonlinear separation still enters crossing")
if "12 * arm" in candidate or "12*arm" in candidate:
    raise RuntimeError(
        f"{CANDIDATE}: retains group-scale rigid support-radius inflation")
for token in (
    "EndpointCorrectedSecondOrderDriftV1",
    "x(u) = (1-u)x0 + u*x1",
    "h^2*|q_i|/8",
    "Every finite binary64 value is an integer multiple of 2^-1074",
    "CertifyRigidPointAffineMotion(",
    "CertifyRigidFacetAffineMotion(",
    "BuildRigidFacetQuadraticCoefficients(",
    "CertifyQuadraticFacetSeparation(",
    "CertifyQuadraticFacetCoverage(",
    "ExactCoefficientBounds(",
    "SubdivideSeparation(",
    "SubdivideCoverage(",
    "LocalSharedVertexOnly(",
    "SharedVertexConeSeparated(",
    "ExactVertexFaceCoplanar(",
    "BernsteinPolynomial",
    "Any nonlocal triangle contact contains a VF or EE feature",
    "intersection_time_numerator",
    "RepresentedFeatureKind::TriangleIntersection",
    "Bernstein convex-hull property",
    "both synchronous children",
    "MissingAcceptedOwner",
    "PossibleGeometricCrossing",
    "represented_q[component]",
    "std::nextafter(",
    "R::RotationLimit",
):
    require(rigid_sweep, token, RIGID_SWEEP)
for token in ("first.certified_affine && second.certified_affine",
              "ExcludedSameRigidGroup"):
    require(arena, token, ARENA)
require(candidate, "motion.certified_affine", CANDIDATE)
if "motion.motion == SelfContactFacetMotion::LinearNodalV1" in candidate:
    raise RuntimeError(
        f"{CANDIDATE}: semantic motion label still selects endpoint prism")
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
for token in (
    "class QualificationAccess",
    "AcceptedEventCertificateView",
    "ClassifyPreparedNonlinearCandidates",
    "ClassifyAcceptedFeaturePolicies",
    "AcceptedFeaturePolicyEvidence",
    "AcceptedFeatureDisposition",
    "NonlinearCandidateRosterSummary",
    "PreparedMotionCertificateView",
    "Impl::Phase::AssemblyRecorded",
):
    require(storage, token, storage_path)
for token in ("accepted_rigid_groups", "prepared_rigid_groups",
              "node_rigid_groups", "parent_motion", "facet_motion",
              "chunk_crossings", "chunk_motion_actions",
              "chunk_feature_task_masks",
              "accepted_event_identities",
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
    "axis_certified_linear_separated",
    "edge_axis_certified_linear_separated",
    "vertex_edge_axis_separated",
    "vertex_vertex_axis_separated",
    "exact_crossing_pairs",
    "exact_crossing_work",
    "feature_task_mask_capacity",
    "feature_task_mask_bytes",
    "max_event_identity_census",
    "accepted_event_identity_census_capacity",
    "event_identity_hash_capacity",
    "AcceptedEventsLowerBound",
    "unsigned discovery_worker_count = 1",
    "unsigned crossing_worker_count = 1",
    "potential_tasks()",
    "local_masked_tasks()",
    "exact_executed_tasks()",
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
filter_header = FILTER_HEADER.read_text()
filter_source = FILTER_SOURCE.read_text()
for token in (
    "CertifiedLinearFacetPrismSeparation(",
    "ProjectionBounds(",
    "CrossAxis(",
    "VertexEdgeAxis(",
    "CrossAxis(vertex_from_start, edge_axis)",
    "Difference(first_vertex, second_vertex)",
    "SelfContactFacetPrismSeparationAxis::EdgeCross",
    "SelfContactFacetPrismSeparationAxis::VertexEdge",
    "SelfContactFacetPrismSeparationAxis::VertexVertex",
    "SelfContactFacetPrismAxisLimit::VertexVertex",
    "std::nextafter(",
    "thickness * norm_l1",
    "first_thickness, second_thickness, axis",
):
    require(filter_source, token, FILTER_SOURCE)
for token in (
    "SelfContactFacetFilterCategory",
    "CoordinateAabbSeparated",
    "FaceAxisSeparated",
    "EdgeCrossAxisSeparated",
    "VertexEdgeAxisSeparated",
    "VertexVertexAxisSeparated",
    "ExactRemaining",
    "ClassifyAcceptedFacetPair(",
):
    require(filter_header + filter_source, token, FILTER_HEADER)
for token in (
    "ExactRemaining = 4",
    "VertexEdgeAxisSeparated = 5",
    "VertexVertexAxisSeparated = 6",
    "Source- and binary-compatible original entry point",
):
    require(filter_header, token, FILTER_HEADER)
for token in (
    "LegacyPrismCertificateEntry",
    "ExtendedPrismCertificateEntry",
    "vertex_edge_axis_separated) == 112",
    "vertex_vertex_axis_separated) == 120",
    "motion_certified_nonlinear_accepted_coverage) == 176",
    "motion_certified_nonlinear_exact_exclusion) == 184",
    "sizeof(c::SelfContactCandidatePolicySummary) == 288",
):
    require(VALUE_TEST.read_text(), token, VALUE_TEST)
require(candidate, "sct::CertifiedLinearFacetPrismSeparation(", CANDIDATE)
for token in (
    "sct::FacetPrismAxisLimit::VertexVertex",
    "summary.vertex_edge_axis_separated",
    "summary.vertex_vertex_axis_separated",
):
    require(candidate, token, CANDIDATE)
for forbidden in ("activity_base", "activity_current"):
    if forbidden in storage or forbidden in arena:
        raise RuntimeError(
            f"{storage_path}: retains duplicate transaction activity storage")

for token in (
    "CopyAccepted",
    "CopyPrepared",
    "state.regularity.Certify",
    "state.candidate_discovery.Discover",
    "state.candidate_discovery.DiscoverMasked",
    "BuildLocalFeatureTaskMasks(",
    "state.crossing.Certify",
    "state.broadphase.Evaluate",
    "ValidateCandidatePublications",
    "FoldPolicyOutcomes(",
    "summary.exact_crossing_pairs += pair_count",
    "summary.exact_crossing_work = crossing_work",
    "participation.SealSelfContactCandidate",
):
    require(candidate, token, CANDIDATE)
for token in (
    "LocallyExcluded(",
    "crossing_pair_count",
    "RepresentedIntervalReason::UnsupportedMotion",
    "Quadratic subdivision and ledger coverage remain unresolved",
    "EvaluatePairFeaturesMaskedOnce(",
    "QuadraticResidualCertificate(",
    "PersistentQuadraticCertificate(",
    "CertifyQuadraticFacetPolicyCoverage(",
    "BuildAcceptedSameRigidExclusions(",
    "CertifiedQuadraticExactExclusion",
    "Quadratic contact cell has no exact accepted VF/EE owner",
):
    require(candidate, token, CANDIDATE)
if "Nonlinear subdivision certificate input is invalid" in candidate:
    raise RuntimeError(
        f"{CANDIDATE}: inconclusive subdivision no longer preserves "
        "UnsupportedMotion")
if candidate.index("state.candidate_discovery.DiscoverMasked") > candidate.index(
        "Quadratic subdivision and ledger coverage remain unresolved"):
    raise RuntimeError(
        f"{CANDIDATE}: unsupported motion precedes exact local geometry")

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
    TASK_MASK,
    ROOT / "lib_src/collision/self_contact_transaction/Source.cpp",
):
    text = path.read_text()
    for forbidden in ("std::vector", "std::map", "std::function",
                      "cudaMalloc", "new "):
        if forbidden in text:
            raise RuntimeError(
                f"{path}: forbidden per-attempt allocation token {forbidden!r}")

source_path = ROOT / "lib_src/collision/self_contact_transaction/Source.cpp"
source = source_path.read_text()
values = (
    ROOT / "lib_src/collision/self_contact_transaction/Values.cpp").read_text()
accepted_builder = source[
    source.index("SelfContactTransactionReport BuildAcceptedEvents("):
    source.index(
        "SelfContactTransactionReport BuildAcceptedSameRigidExclusions(")]
if accepted_builder.count(
        "for (std::size_t feature = 0; feature < features.count; ++feature)") != 1:
    raise RuntimeError(
        f"{source_path}: accepted event construction is not one feature pass")
for token in (
    "ActiveUseQueryAccess::ValidateActivity(",
    "const bool remaining = written < capacity",
    "remaining ? events + written : nullptr",
    "written += admitted && remaining",
    "if (required > capacity)",
):
    require(accepted_builder, token, source_path)
if accepted_builder.count(
        "ActiveUseQueryAccess::ValidateActivity(") != 1:
    raise RuntimeError(
        f"{source_path}: accepted chunk does not authenticate activity once")
for token in (
    "ActiveUseQueryAccess::ClassifyVertexFace(",
    "ActiveUseQueryAccess::ClassifyEdgeEdge(",
    "SeparatedFromForceSupport(feature, classification, &separated)",
    "gap > feature.representation_error_m",
):
    require(source, token, source_path)
for text, path in ((transaction, TRANSACTION), (candidate, CANDIDATE)):
    require(text, "broadphase.required_pairs", path)
require(source, "ClassifyAcceptedFacetPair(", source_path)
for token in (
    "BuildFixedTriangleFeatureTaskMask(",
    "CurrentFixedTriangle Identity(",
    "Validate the complete chunk before publishing any mask",
):
    require(task_mask, token, TASK_MASK)
for forbidden in (
    "same_pid", "tied", "rigid", "regularity", "coordinate",
):
    if forbidden in task_mask.lower():
        raise RuntimeError(
            f"{TASK_MASK}: local mask depends on forbidden remote policy {forbidden!r}")

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
    "HashSelfContactForceEventIdentity",
    "SelfContactForceEventIdentityOf",
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
    "input.crossings.data[pair].reason ==",
    "LocallyExcluded(\n              input.intersections",
    "CertifyLinearResidualSeparation(",
    "TriangleResidualL1(",
    "SquaredDistance(",
    "Multiply(margin, margin)",
    "FixedTriangleFeatureTaskBits",
    "CertifyPersistentLinearContact(",
    "ExactFeatureSquaredDistance(",
    "normalized_weights[adjusted]",
    "DyadicUpper(normalization_error)",
    "Multiply(available, available)",
    "first_owner <= 0 && second_owner <= 0",
    "crossing_target < 2",
    "CertifyQuadraticResidualSeparation(",
    "QuadraticChordDeviationL1Upper(",
    "CertifyPersistentQuadraticContact(",
    "AcceptedCoverageFeature(",
    "PersistentAcceptedLedgerCoverage",
):
    require(values, token, storage_path)
for token in (
    "CertifiedResidualLinearSeparation",
    "CertifyLinearResidualSeparation(",
    "local_result.work = 1",
    "CertifiedPersistentLinearContact",
    "PersistentLinearCertificate(",
    "PersistentPhysicalContact",
    "CertifiedQuadraticResidualSeparation",
    "CertifiedPersistentQuadraticContact",
    "ClassifyPreparedCandidateCensus(",
    "LinearWorkExhaustedRosterEntry",
    "LinearCandidateCensusSummary",
    "Linear policy coverage",
    "CertifyQuadraticFacetPolicyCoverage(",
    "linear_policy_potential_contact",
    "linear_policy_possible_geometric_crossing",
):
    require(candidate + storage, token, CANDIDATE)
for token in (
    "unresolved_path",
    "unresolved_depth",
    "has_unresolved_cell",
):
    require(rigid_sweep + storage, token, RIGID_SWEEP)

for wiring in (CMAKE, BAZEL):
    text = wiring.read_text()
    for token in ("self_contact_transaction/Arena.cpp",
                  "self_contact_transaction/Candidate.cpp",
                  "self_contact_transaction/Limits.cpp",
                  "self_contact_transaction/RigidSweep.cpp",
                  "self_contact_transaction/Source.cpp",
                  "self_contact_transaction/Streaming.cpp",
                  "self_contact_transaction/TaskMask.cpp",
                  "self_contact_transaction/Transaction.cpp",
                  "self_contact_transaction/Values.cpp"):
        require(text, token, wiring)
    require(text, "self_contact_physical_activity", wiring)
    require(text, "self_contact_filter_certificates", wiring)

for token in (
    "SELF_CONTACT_TRANSACTION_CUDA",
    "CudaTest.cu",
    "RigidSweepBoundsTest.cpp",
    "AcceptedEventTest.cpp",
    "MediumCouponTest.cpp",
    "self_contact_transaction_medium_coupon",
    "tl_self_contact_transaction",
    "rigid-cin-response",
    'LABELS "unit;',
    'LABELS "coupon;',
):
    require(QUAL_CMAKE.read_text(), token, QUAL_CMAKE)
for token in ("host_check", "medium_coupon", "source_check",
              "root_cuda_sources", "AcceptedEventTest.cpp"):
    require(QUAL_BAZEL.read_text(), token, QUAL_BAZEL)
require(QUAL_BAZEL.read_text(), '":root_cuda_sources"', QUAL_BAZEL)
require(QUAL_BAZEL.read_text(), "m2-rigid-cin-response", QUAL_BAZEL)
require(QUAL_CMAKE.read_text(), "symmetric-edge-area", QUAL_CMAKE)
require(QUAL_BAZEL.read_text(), "symmetric-edge-area", QUAL_BAZEL)
require(QUAL_CMAKE.read_text(), "prism-axis-certificate", QUAL_CMAKE)
require(QUAL_BAZEL.read_text(), "prism-axis-certificate", QUAL_BAZEL)
require(QUAL_CMAKE.read_text(), "local-feature-task-mask", QUAL_CMAKE)
require(QUAL_BAZEL.read_text(), "local-feature-task-mask", QUAL_BAZEL)
require(QUAL_CMAKE.read_text(), "exact-event-census", QUAL_CMAKE)
require(QUAL_BAZEL.read_text(), "exact-event-census", QUAL_BAZEL)
for token in (
    "DecisionCount = 262144",
    "ChunkCapacity = 257",
    "GeometryPairCount = 12800",
    "ExpectedPolicyDigest",
    "ClassifyCandidatePairMotion(",
    "CertifiedLinearFacetPrismSeparation(",
    "geometry_baseline",
    "geometry_face_axes",
    "geometry_edge_axes",
    "geometry_vertex_edge_axes",
    "geometry_vertex_vertex_axes",
    "vertex_edge_axes.exact_discovery_tasks",
    "vertex_vertex_axes.crossing_work",
    "geometry_local_mask",
    "geometry_local_mask_worker4",
    "geometry_local_mask_crossing_worker4",
    "crossing_digest",
    "local_mask.local_masked_tasks",
    "local_mask.feature_events",
    "parallel_discovery_limits.worker_count = 4",
    "feature_digest",
    "intersection_digest",
    "ExpectSameGeometryObservations",
    "ClosestFeatureAxesAreIncrementalAndSweepConservative",
    "MergeAcceptedEventChunk(",
    "FinalizeAcceptedEventLedger(",
    "summary.digest, ExpectedPolicyDigest",
    "sizeof(FixedStorage) < 512u * 1024u",
):
    require(MEDIUM.read_text(), token, MEDIUM)
for token in (
    "MillionSyntheticIdentitiesUseBoundedCompactStorage",
    "sizeof(c::SelfContactForceEventIdentity) == 240",
    "AcceptedEventsLowerBound",
    "MergeAcceptedEventIdentityChunk(",
    "ResidualTranslationUsesOutwardBoundsAfterCancellation",
    "ResidualTranslationRetainsSubnormalExactMotion",
    "ResidualTranslationIsInvariantToVertexPermutation",
    "ResidualTranslationFailsClosedOnOverflow",
    "ResidualTranslationPreservesContactAndUnequalMotion",
    "ResidualTranslationSubtractsRepresentationErrorStrictly",
    "QuadraticResidualSubtractsOutwardChordDeviation",
    "PersistentQuadraticContactRequiresCurvatureMargin",
    "QuadraticLedgerCoverageSubdividesAndFailsClosed",
    "QuadraticSharedVertexNoRootCertificateMatchesDyadicOracle",
    "QuadraticSharedVertexExactEndpointRootIsCanonical",
    "QuadraticSharedVertexRootBoundaryDegeneracyAndCapsFailClosed",
    "QuadraticSharedVertexPermutationAndRetryPreserveInputs",
    "TinyDyadicQuadraticCoverageOracleIsExhaustive",
    "BoundaryVertexLedgerAndSameRigidExclusionCoverSharedEdge",
    "ExactCommonMotionPublishesMatchingPersistentEdgeEdge",
    "ExactCommonMotionPublishesMatchingPersistentVertexFace",
    "PersistentVertexFaceNormalizesDyadicWeightsExactly",
    "PersistentVertexFaceNormalizesOnlyCanonicalSourceOwner",
    "PersistentContactRejectsFeatureSwitchAndSeamIdentity",
    "PersistentContactRejectsCrossingAndContactLoss",
    "PersistentContactUsesStrictNearThresholdBound",
    "PersistentContactCanonicalizesEqualMinimaAcrossPermutation",
    "PersistentPublicationRequiresExactAcceptedCertificate",
):
    require(VALUE_TEST.read_text(), token, VALUE_TEST)
for token in (
    "ZeroSpinIsTightOutwardRoundedTranslation",
    "TinyAngleInflationScalesQuadraticallyPerMember",
    "EndpointsAndCompleteCertifiedQuadraticStayContained",
    "UnsupportedRotationAndNonfiniteInputsLeaveOutputUnchanged",
    "MotionCertificateIsPartOfPreparedOwnerIdentity",
    "ExactRepresentedAffineCertificateComposesWeightedCurvature",
    "DyadicQuadraticSubdivisionIsConservativeBoundedAndSymmetric",
    "PotentialNonlinearContactNeverCertifiesSeparated",
    "LinearVersusQuadraticPathCertifiesWholeIntervalSeparation",
    "SmallDyadicOracleNeverFindsContactBehindSeparation",
    "CertifyRigidPointAffineMotion(",
    "CertifyRigidFacetAffineMotion(",
):
    require(RIGID_SWEEP_TEST.read_text(), token, RIGID_SWEEP_TEST)
for token in (
    "OnePrivatePassRetainsLateFailurePriorityAndCountPublication",
    "ShortPrivateCapacityCountsCompletelyWithoutOverwritingGuard",
    "EXPECT_EQ(count, 777u)",
    "EXPECT_EQ(count, 888u)",
    "EXPECT_EQ(short_report.candidate, 2u)",
):
    require(ACCEPTED_TEST.read_text(), token, ACCEPTED_TEST)

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
    "CertifiedRigidSweepsSeparateDistantBodiesButNotOverlappingArcs",
    "ExactAffineMixedCertificateAndDecisionAreRepeatable",
    "NonlinearSubdivisionDecisionIsRepeatableAndFailClosed",
    "QuadraticSharedVertexCertificateIsRepeatedlyBitwiseStable",
    "QuadraticResidualCertificateIsBitwiseRepeatable",
    "CertifyQuadraticFacetCoverage(",
    "CommonTranslationCertificateIsDeterministicAtMinimalCap",
    "ResidualTranslationCertificateIsDeterministic",
    "CertifyPersistentLinearContact(",
    "ExactLocalIntersectionPrecedesOnlyUnsupportedMotion",
    "ContactConstraintLayout::SameMergedParts",
    "ContactConstraintLayout::MergedPartAndPlain",
    "CopyPreparedForceStage",
    "endpoint_inverse_sum",
    "PairMotionAction::UnsupportedRigidArc",
    "PairMotionAction::CertifiedRigidArcSeparation",
    "common, prepared, accepted",
    "removing_parents()",
    "skipped_parents()",
    "boundary_vertex_edge_event_count",
    "motion_certified_linear_separated",
    "axis_certified_linear_separated",
    "edge_axis_certified_linear_separated",
    "vertex_edge_axis_separated",
    "vertex_vertex_axis_separated",
    "exact_crossing_pairs",
    "exact_crossing_work",
    "accepted.potential_tasks()",
    "accepted.local_masked_tasks()",
    "accepted.exact_executed_tasks()",
    "receipt.potential_tasks()",
    "receipt.local_masked_tasks()",
    "receipt.exact_executed_tasks()",
):
    require(cuda, token, CUDA)

# Determinism is an algorithm/source contract, not a conclusion drawn from one
# CUDA run. The only self-contact broadphase atomic is an order-independent
# integer minimum; accepted force/STI contains no atomics of any kind.
broadphase_kernels_path = (
    ROOT / "lib_src/collision/self_contact_broadphase/Kernels.cu")
broadphase_sort_path = (
    ROOT / "lib_src/collision/self_contact_broadphase/Sort.cu")
broadphase_layout_path = (
    ROOT / "lib_src/collision/self_contact_broadphase/Layout.h")
force_operations_path = (
    ROOT / "lib_src/collision/self_contact_force/Operations.cu")
force_values_path = (
    ROOT / "lib_src/collision/self_contact_force/Values.cpp")
discovery_path = (
    ROOT / "lib_src/collision/fixed_triangle_features/Discovery.cpp")
crossing_path = ROOT / "lib_src/collision/RepresentedIntervalCrossing.cpp"
participation_path = (
    ROOT / "lib_src/elements/publication/"
    "ShellPhysicalScratchParticipation.cpp")

broadphase_kernels = broadphase_kernels_path.read_text()
broadphase_sort = broadphase_sort_path.read_text()
broadphase_layout = broadphase_layout_path.read_text()
force_operations = force_operations_path.read_text()
force_values = force_values_path.read_text()
discovery_source = discovery_path.read_text()
crossing_source = crossing_path.read_text()
participation_source = participation_path.read_text()
atomic_call = re.compile(r"\batomic[A-Za-z0-9_]*\s*\(")
if atomic_call.findall(force_operations):
    raise RuntimeError(
        f"{force_operations_path}: accepted force/STI uses an atomic")
if atomic_call.findall(broadphase_kernels) != ["atomicMin("]:
    raise RuntimeError(
        f"{broadphase_kernels_path}: unexpected broadphase atomic set")
for token in (
    "std::uint32_t invalid_parent",
    "std::uint64_t count",
):
    require(broadphase_layout, token, broadphase_layout_path)
for token in (
    "atomicMin(&control->invalid_parent",
    "keys[offset++] =",
    "WriteCanonical output{keys, offsets[i]}",
    "VisitLater(boxes, n, i, axis",
):
    require(broadphase_kernels, token, broadphase_kernels_path)
for token in (
    "DeviceRadixSort::SortPairs",
    "DeviceScan::ExclusiveSum",
    "DeviceRadixSort::SortKeys",
):
    require(broadphase_sort, token, broadphase_sort_path)

for token in (
    "std::sort(events, events + event_count, EventLess)",
    "std::sort(incidences, incidences + incidence_count, IncidenceLess)",
):
    require(force_values, token, force_values_path)
for token in (
    "One CUDA thread owns each row",
    "folds its incidences in that canonical order",
    "Floating atomics are",
    "Every transfer and kernel below is submitted to the authenticated owner",
    "disjoint writers before its single-thread canonical checker/reducer",
):
    require(force_operations, token, force_operations_path)
launches = (
    "Begin<<<", "EvaluateEvents<<<", "CheckEvents<<<",
    "ReduceDiagnostics<<<", "StageNodes<<<", "CheckNodes<<<",
    "PublishNodes<<<", "Finish<<<")
cursor = -1
for launch in launches:
    next_cursor = force_operations.index(launch)
    if next_cursor <= cursor:
        raise RuntimeError(
            f"{force_operations_path}: CUDA phases are not in fixed order")
    cursor = next_cursor
if force_operations.count(", state.stream>>>") < 8:
    raise RuntimeError(
        f"{force_operations_path}: force phases do not share one stream")

for text, path, writer_proof in (
    (discovery_source, discovery_path, "one writer"),
    (crossing_source, crossing_path, "one staging/status writer"),
):
    for token in (
        "next_pair.fetch_add(1, std::memory_order_relaxed)",
        writer_proof,
        "canonical",
    ):
        require(text, token, path)
    if text.index("RunWorkers(") >= text.rindex("std::sort"):
        raise RuntimeError(
            f"{path}: worker results are not canonically reduced")
for token in (
    "for (std::size_t slot=0;slot<PhysicalScratchKindCount;++slot)",
    "issuer.stream_=view.stream",
    "issuer.stream_!=authentic.stream",
):
    require(participation_source, token, participation_path)
for token in (
    "CUDA execution order is inherently nondeterministic",
    "algorithmic rules define bitwise",
):
    require(header, token, HEADER)
for token in (
    "AlgorithmicDeterminismAcrossSchedulingWorkersAndLifetimes",
    "for (unsigned repetition = 0; repetition < 32; ++repetition)",
    "constexpr unsigned Workers[]{1, 2, 4}",
    "PriorStreamWork",
    "cudaStreamWaitEvent",
    "PolicyOutcomeBits",
    "canonical_event_order",
    "p::Exact(initial, rolled_back)",
    "new (&fixture_storage) Fixture",
):
    require(cuda, token, CUDA)
for token in (
    "self_contact_determinism_cuda",
    'LABELS "coupon;determinism;cuda"',
    "TIMEOUT 600",
):
    require(QUAL_CMAKE.read_text(), token, QUAL_CMAKE)

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
