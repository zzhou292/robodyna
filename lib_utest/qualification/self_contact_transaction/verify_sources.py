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
filter_source = (FILTER_SOURCE.read_text() +
                 FILTER_SOURCE.with_name("self_contact_filters").joinpath("Prism.h").read_text() +
                 FILTER_SOURCE.with_name("self_contact_filters").joinpath("Arithmetic.h").read_text())
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
    "::nextafter(",
    "thickness * norm_l1",
    "first_thickness, second_thickness, axis",
):
    require(filter_source, token, FILTER_SOURCE)
# Shared HD adapters preserve explicit outward directions; the owning filter
# extraction gate additionally pins all20 arithmetic bodies to21941d20.
shared_arithmetic = FILTER_SOURCE.with_name("self_contact_filters").joinpath("Arithmetic.h").read_text()
for signature, direction in (("double Down(double value)", "-HUGE_VAL"),
                             ("double Up(double value)", "HUGE_VAL")):
    begin = shared_arithmetic.index(signature)
    body = shared_arithmetic[shared_arithmetic.index("{", begin)+1:shared_arithmetic.index("}", begin)]
    assert re.sub(r"\s+", "", body) == "return::nextafter(value," + direction + ");"
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

prism_impl = filter_source[filter_source.index("bool CertifiedLinearFacetPrismSeparationImpl("):
                           filter_source.index("}  // namespace", filter_source.index("bool CertifiedLinearFacetPrismSeparationImpl("))]
assert prism_impl.index("if (!valid)") < prism_impl.index("*separated_axis =")
assert prism_impl.index("if (!*valid)") < prism_impl.index("EndpointHullsShareVertex<observe>(")
assert prism_impl.index("EndpointHullsShareVertex<observe>(") < prism_impl.index("const Vec3 axes[4]")
assert "CertifiedLinearFacetPrismSeparationImpl<true, false>" in filter_source
assert "CertifiedLinearFacetPrismSeparationImpl<false, true>" in filter_source

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
    "sct::CertifyCrossingBatches",
    "state.broadphase.Evaluate",
    "ValidateCandidatePublications",
    "FoldPolicyOutcomes(",
    "summary.exact_crossing_pairs += pair_count",
    "summary.exact_crossing_work = crossing_work",
    "participation.SealSelfContactCandidate",
):
    require(candidate, token, CANDIDATE)
for token in (
    "crossing_pair_count",
    "RepresentedIntervalReason::UnsupportedMotion",
    "Quadratic subdivision and ledger coverage remain unresolved",
    "EvaluatePairFeaturesMaskedOnce(",
    "QuadraticResidualCertificate(",
    "CertifyQuadraticFacetPolicyCoverage(",
    "exclusion_context.source()",
    "CertifiedQuadraticExactExclusion",
    "CertifiedQuadraticLocalIntersection",
    "CertifiedLocalTopology",
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
accepted_fold_path = ROOT / "lib_src/collision/self_contact_transaction/AcceptedFacetFiltering.h"
accepted_fold = accepted_fold_path.read_text()
require(source, "detail::FilterAcceptedFacetPairsWith(active_use, triangles, motion,", source_path)
require(source, "&::tlfea::contact::ClassifyAcceptedFacetPair", source_path)
for token in ("ClassifyAcceptedFacetPair(", "first_parent >= parents.size()",
              "filtered.status != SelfContactFacetFilterStatus::Ok",
              "SelfContactFacetFilterCategory::ExactRemaining", "pairs[write++] = value"):
    require(accepted_fold, token, accepted_fold_path)
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
    "Endpoint-local intersection lacks continuous topology proof",
    "Continuous local topology publication lacks its exact local premise",
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
    "ExactAffineVertexFaceTransition(",
    "ExactBernsteinSign(",
    "transition_time_lower_numerator",
    "transition_time_exact",
    "transition_zero_geometry_separated",
    "closed_covered_cells",
):
    require(rigid_sweep + storage, token, RIGID_SWEEP)

for wiring in (CMAKE, BAZEL):
    text = wiring.read_text()
    for token in ("self_contact_transaction/Arena.cpp",
                  "self_contact_transaction/Candidate.cpp",
                  "self_contact_transaction/CandidateFailureCapture.cpp",
                  "self_contact_transaction/Limits.cpp",
                  "self_contact_transaction/RigidSweep.cpp",
                  "self_contact_transaction/Source.cpp",
                  "self_contact_transaction/FacetFilterValues.cpp",
                  "self_contact_transaction/FacetFilters.cpp",
                  "self_contact_transaction/FacetFilterAccepted.cpp",
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
transaction_fixture = CUDA.parent / "TransactionFixture.h"
require(cuda, '#include "TransactionFixture.h"', CUDA)
for wiring in (QUAL_CMAKE, QUAL_BAZEL):
    require(wiring.read_text(), "TransactionFixture.h", wiring)
cuda += "\n" + transaction_fixture.read_text()
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
    "LocalEndpointCannotAdmitUnsupportedContinuousMotion",
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


# Qualification-only evidence observes the exact production failure before
# rollback. It cannot route around policy or publish a candidate.
capture_path = ROOT / "lib_src/collision/self_contact_transaction/CandidateFailureCapture.cpp"
capture = capture_path.read_text()
for token in ("SealCandidateImpl(", "ValidateQualificationRanges(outputs, inputs,",
              "capture.activity.activity_ = activity", "observer->capture(observer->context, capture)"):
    require(capture, token, capture_path)
for token in ("assembly, output, nullptr)",
              "QualificationAccess::ObserveCandidateFailure("):
    require(candidate, token, CANDIDATE)
for token in ("capture.has_nonlinear_baseline = true", "capture.nonlinear_baseline = *nonlinear_baseline",
              "ObserveCandidateFeatureFailure(", "std::lower_bound(begin, end, key"):
    require(capture, token, capture_path)
require(candidate, "assembly, activity_receipt, &nonlinear)", CANDIDATE)
require(candidate, "QualificationAccess::ObserveCandidateFeatureFailure(", CANDIDATE)
nonlinear_failure_cases = Path(__file__).resolve().parent / "NonlinearFailureCaptureCases.h"
for token in ("NonlinearTerminalObserverKeepsActualCurvatureRejectionAndRollback",
              "ContactConstraintLayout::MergedPartAndPlain", "fixture.rig.external_force_z_n = 1000",
              "max_nonlinear_subdivision_work_per_pair = 1", "max_nonlinear_subdivision_depth = 0",
              "has_nonlinear_baseline", "observation.has_curvature", "std::bad_alloc{}",
              "facet_capacity < 2", "p::Exact(before, after)"):
    require(nonlinear_failure_cases.read_text(), token, nonlinear_failure_cases)
failure_cases = Path(__file__).resolve().parent / "FailureCaptureCases.h"
for token in ("CandidateFailureObserverSeesLiveSourceAndPreservesReportAndRollback",
              "CandidateFailureObserverDoesNotRunOnSuccessOrUnauthenticatedFailure",
              "CandidateFailureObserverRejectsAliasedAndOverflowedContextBeforeWrites",
              "p::Exact(before, after)", "ExactReport(report, baseline)"):
    require(failure_cases.read_text(), token, failure_cases)

# Continuous local policy requires a real interval proof, never the old
# endpoint-only exception. The runtime tests exercise actual geometry/owners.
local_path = ROOT / "lib_src/collision/self_contact_transaction/LocalContact.cpp"
local_source = local_path.read_text()
for token in ("CertifyQuadraticUnmaskedSeparation(",
              "CertifyQuadraticLocalTopology(",
              "EvaluatePairFeaturesMaskedOnce("):
    require(local_source, token, local_path)
for token in ("CertifiedLocalIntersection", "lower_local && upper_local",
              "local.work >= max_work", "max_work - local.work"):
    require(rigid_sweep, token, RIGID_SWEEP)
if "LocalPolicyResolvesUnsupported" in candidate:
    raise RuntimeError(f"{CANDIDATE}: endpoint-only local bypass returned")
if "PersistentQuadraticCertificate(" in candidate:
    raise RuntimeError(f"{CANDIDATE}: thickness-only curved bypass returned")
for build_path in (CMAKE, BAZEL):
    require(build_path.read_text(), "self_contact_transaction/LocalContact.cpp", build_path)

local_cases = Path(__file__).resolve().parent / "LocalPublicationCases.h"
for token in ("AdjacentLocalFirstIntervalCommitsThenSecondIntervalRejectsRetryAtomically",
              "fixture.transaction.SealCandidate(",
              "EdgeInteriorRepresentation", "EXPECT_FALSE(receipt.valid())",
              "p::Exact(before, after)", "EXPECT_EQ(geometry, first_geometry)",
              "CheckSecondIntervalRepresentationCoordinates(prepared_facets)",
              "fixture.Commit(token,prepared,common,receipt)"):
    require(local_cases.read_text(), token, local_cases)
# The finite cone-axis search may add candidates, never authority: preserve
# the old order and the same interval sign check within the existing storage.
for token in ("constexpr Vec3 coordinate_diagonals[]",
              "3 + 4 + 6 + 1 + 6 <= 32",
              "for (const auto axis : coordinate_diagonals)",
              "DotPolynomialAxis(", "StrictPolynomialOrientation(projection)"):
    require(rigid_sweep, token, RIGID_SWEEP)
cone_cases = Path(__file__).resolve().parent / "ConeDiagonalCases.h"
for token in ("CapturedAffineCoordinatesNeedOneRootProofWithoutSourceSpecialCases",
              "AllVertexOrdersRetainSharedIdentityAndRootProof",
              "SignedCoordinatePermutationsAndQuarterTurnsRetainRootProof",
              "DyadicBoundaryRequiresStrictConeSeparation",
              "RotatedActualRigidMidintervalCrossingRemainsRejected",
              "MalformedSharedTrajectoryDegeneracyAndZeroWorkFailClosed",
              "CertifyQuadraticLocalTopology(", "RequiresIntersectionAdmission(intersection)"):
    require(cone_cases.read_text(), token, cone_cases)
require((cone_cases.parent / "ContinuousLocalTest.cpp").read_text(),
        '#include "ConeDiagonalCases.h"', cone_cases)
for name in ("CMakeLists.txt", "BUILD.bazel"):
    require((cone_cases.parent / name).read_text(), cone_cases.name,
            cone_cases.parent / name)

# The original accepted-contact CUDA test retains the positive two-commit
# lifecycle. The adjacent local coupon qualifies its first commit followed by
# the exact second-interval representation rejection and rollback/retry.
for token in ("AcceptedInteriorEeForceCandidateRetryAndRollbackKeepForceSti",
              "for (unsigned interval = 0; interval < 2; ++interval)",
              "fixture.Commit(token, prepared, common, receipt)"):
    require(CUDA.read_text(), token, CUDA)

exclusion_path = ROOT / "lib_src/collision/self_contact_transaction/CandidateExclusions.cpp"
for token in ("EvaluatePairFeaturesMaskedOnce(", "BuildAcceptedSameRigidExclusions("):
    require(exclusion_path.read_text(), token, exclusion_path)
for token in ("deferred_exclusions->prepare(", "deferred_exclusions->report",
              "exclusion_count > deferred_exclusions->capacity"):
    require(rigid_sweep, token, RIGID_SWEEP)

rigid_feature_cases = Path(__file__).resolve().parent / "RigidFeatureGeometryCases.h"
require(rigid_feature_cases.read_text(),
        "MixedFacetCannotHideInteriorCrossingBehindStationaryRigidVertexFace",
        rigid_feature_cases)
require(rigid_sweep, "max_work - ledger.work, max_depth, true)", RIGID_SWEEP)
if "max_work - ledger.work, max_depth, false)" in rigid_sweep:
    raise RuntimeError(f"{RIGID_SWEEP}: rigid feature waives whole-facet geometry")

# Native exact translation is a whole-interval geometry premise, consumed
# only after the transaction's unchanged physical and current-source gates.
translated_path = ROOT / "lib_src/collision/self_contact_transaction/TranslatedLocal.cpp"
translated = translated_path.read_text()
for token in ("HasExactCommonTranslationProof(result->geometry)",
              "!intersections.complete", "result->work != 1",
              "FindPairIntersection(intersections, result->key)",
              "RequiresIntersectionAdmission(*intersection)",
              "RepresentedIntersectionGeometry::CertifiedLocalTopology",
              "result->accepted_event = SIZE_MAX"):
    require(translated, token, translated_path)
for token in ("NormalizeExactTranslatedLocal(",
              "translated_local != sct::TranslatedLocalStatus::Certified"):
    require(candidate, token, CANDIDATE)
if candidate.index("ValidateCandidateEdgePolicy(") > candidate.index("NormalizeExactTranslatedLocal("):
    raise RuntimeError(f"{CANDIDATE}: translated local normalization precedes native edge policy")
for build_path in (CMAKE, BAZEL):
    require(build_path.read_text(), "self_contact_transaction/TranslatedLocal.cpp", build_path)
translated_cases = Path(__file__).resolve().parent / "TranslatedLocalCases.h"
for token in ("NativeStaticAndTranslatedLocalProofPreservesWorkAndPublishesLocalGeometry",
              "TranslatedNonlocalGeometryCannotBorrowLocalPublication",
              "LocalNormalizationCannotPromoteOrdinaryWitnessOrUnsupportedMotion",
              "LocalNormalizationRequiresCompleteMatchingNativeInputsWithoutPartialWrite",
              "TranslationProofDoesNotWeakenStandaloneUnownedThicknessContract"):
    require(translated_cases.read_text(), token, translated_cases)

translated_cuda = Path(__file__).resolve().parent / "TranslatedLocalCudaCases.h"
for token in ("OriginalAdjacentStaticAndUniformStartupUseTheirActualLocalProofs",
              "ReferenceOnsetLocalContactRetainsPositiveForceAndFreeSti",
              "p::Bits(point.z), p::Bits(p::H)", "diagnostics.maximum_force_norm_n",
              "diagnostics.maximum_represented_stiffness_n_m",
              "ASSERT_FALSE(positive_owner_pairs.empty())",
              "EXPECT_GT(force_pairs_on_local_path, 0u)",
              "EXPECT_GT(force_change, 0)", "p::Exact(initial, discarded)",
              "fixture.Commit(token, prepared, common, receipt)"):
    require(translated_cuda.read_text(), token, translated_cuda)

batch_path = ROOT / "lib_src/collision/self_contact_transaction/CrossingBatch.cpp"
batch = batch_path.read_text()
batch_execution_path = ROOT / "lib_src/collision/represented_interval_crossing/BatchExecution.h"
batch_execution = batch_execution_path.read_text()
require(batch, "represented_interval_crossing::BatchAccess::Certify(", batch_path)
require(batch, "CrossingBatchDiagnostics(", batch_path)
for token in ("pairs ? pairs + report.batch_offset",
              "scratch[report.batch_offset + pair] = current.data[pair]",
              "report.results = {scratch, pair_count, true}",
              "compare(preceding, key) >= 0"):
    require(batch_execution, token, batch_execution_path)
for inspected, path in ((batch, batch_path), (batch_execution, batch_execution_path)):
    for forbidden in ("new ", "malloc(", "reserve(", "resize(", "push_back("):
        if forbidden in inspected:
            raise RuntimeError(f"{path}: per-call allocation {forbidden}")
for token in ("crossing_batch_pair_capacity", "raw_crossing_result_capacity",
              "raw_crossing_result_bytes"):
    require(TYPES.read_text(), token, TYPES)
for token in ("sct::CrossingBatchDiagnostics(crossing)",
              "Linear qualification represented stream exceeds its hard work cap",
              "state.buffers.chunk_raw_crossings"):
    require(candidate, token, CANDIDATE)
for build_path in (ROOT / "lib_src/collision/BUILD.bazel",
                   ROOT / "lib_src/collision/SelfContactTransaction.cmake",
                   Path(__file__).resolve().parent / "CMakeLists.txt"):
    require(build_path.read_text(), "self_contact_transaction/CrossingBatch.cpp", build_path)

# Range lookup reuses transaction-owned finalized order without allowing callers
# to claim that an arbitrary array is sorted. Runtime differential tests are the
# semantic oracle; these checks pin the production authority/lifetime placement.
ledger_path = RIGID_SWEEP.with_name("FinalizedCoverageLedger.h")
ledger = ledger_path.read_text()
for token in ("friend class ::tlfea::contact::SelfContactTransaction",
              "FinalizedCoverageLedger(const FinalizedCoverageLedger&) = delete",
              "std::array<Range, 12>", "fixed_triangle_features::Compare",
              "feature.vertex_face.vertex", "feature.edge_edge.edges[0]"):
    require(ledger, token, ledger_path)
require(candidate, "const sct::FinalizedCoverageLedger coverage_ledger(", CANDIDATE)
if candidate.index("const sct::FinalizedCoverageLedger coverage_ledger(") < candidate.index("!same_assembly"):
    raise RuntimeError(f"{CANDIDATE}: finalized ledger borrowed before assembly authentication")
for token in ("finalized->ForPair(prepared_triangles)",
              "ranges.values[0] = {0, accepted_count}",
              "BuildCoverageOwner(", "owner_count == MaximumOwners",
              "certificate, &owner", "owners[owner - 1].source_order =="):
    require(rigid_sweep, token, RIGID_SWEEP)

# Fixed production order and private legacy-order qualification are distinct.
for token in ("enum class SharedVertexOrder { PolynomialFirst, ConeFirst }",
              "if constexpr (order == SharedVertexOrder::ConeFirst)",
              "if constexpr (order == SharedVertexOrder::PolynomialFirst)",
              "CertifyQuadraticLocalTopologyImpl<SharedVertexOrder::ConeFirst>",
              "CertifyQuadraticLocalTopologyImpl<SharedVertexOrder::PolynomialFirst>",
              "SubdivideCoverage<order, search>", "LocalSharedVertexOnly<order>"):
    require(rigid_sweep, token, RIGID_SWEEP)
local_proof = rigid_sweep[rigid_sweep.index("bool LocalSharedVertexOnly("):
                          rigid_sweep.index("bool LocalSharedEdgeOnly(")]
if not (local_proof.index("shared_vertex_endpoint(lower_triangles)") <
        local_proof.index("!SameCoordinatePath(") <
        local_proof.index("!FacetNondegenerate(") <
        local_proof.index("SharedVertexOrder::ConeFirst") <
        local_proof.index("bool no_nonlocal_root = true")):
    raise RuntimeError(f"{RIGID_SWEEP}: proof-order change bypasses original premises")
qualification_header = RIGID_SWEEP.with_name("SharedVertexProofQualification.h").read_text()
for token in ("SharedVertexProofOrderComparison", "CompareSharedVertexTopologyOrders",
              "CompareSharedVertexCoverageOrders", "SharedVertexProofCounters"):
    require(qualification_header, token, RIGID_SWEEP)
if "SharedVertexOrder" in qualification_header:
    raise RuntimeError(f"{RIGID_SWEEP}: private proof order escaped as caller configuration")

# Streamed candidates never replace the existing strict whole-cell verifier.
cone_direction_path = RIGID_SWEEP.with_name("ConeDirections.h")
cone_direction = cone_direction_path.read_text()
for token in ("ConeDirections::MaximumDirections == 92", "CurvedConeDirections::MaximumDirections == 298", "std::array<Vec3, RayCount>",
              "edge, geometry_detail::Cross"):
    require(cone_direction, token, cone_direction_path)
for token in ("SharedVertexAxisSeparated(facets, shared, remote, axis)",
              "search != RootConeSearch::Original && depth == 0 && path == 0",
              "search == RootConeSearch::Full && depth == 0 && path == 0",
              "local_topology_only && exact_affine", "local_topology_only && !exact_affine",
              "if (!lower_local || !upper_local) return false",
              "RootConeSearch::Original", "RootConeSearch::AffineOnly", "CompareAffineConeSearch",
              "CompareCurvedConeSearch", "CurvedRootConeSeparated",
              "SharedVertexProofCounters::curved_directions"):
    require(rigid_sweep, token, RIGID_SWEEP)
if not (local_proof.index("if (no_nonlocal_root && lower_local && upper_local) return true") <
        local_proof.index("AffineRootConeSeparated(")):
    raise RuntimeError("Generic cone search displaced an existing polynomial success")

curved_search = rigid_sweep[rigid_sweep.index("bool CurvedRootConeSeparated("):
                            rigid_sweep.index("bool FacetNondegenerate(")]
for token in ("control < 3", "PolynomialVectorDifference(",
              ".5 * value.lower + .5 * value.upper",
              "SharedVertexAxisSeparated(facets, shared, remote, axis)"):
    require(curved_search, token, RIGID_SWEEP)
if "RootConeSearch" in qualification_header:
    raise RuntimeError("Root cone proof mode escaped as caller configuration")

# Sorted lookup is a private lexical borrow of an authenticated completed
# discovery cohort. Public raw arrays retain their original first-match scan.
sorted_path = translated_path.with_name("SortedIntersections.h")
sorted_header = sorted_path.read_text()
for token in ("friend class ::tlfea::contact::SelfContactTransaction",
              "SortedIntersections(const SortedIntersections&) = delete",
              "SortedIntersections(SortedIntersections&&) = delete",
              "explicit SortedIntersections(const FixedTriangleFeatureDiscovery&)"):
    require(sorted_header, token, sorted_path)
for token in ("!fixed_triangle_features::IntersectionLess(view.data[row - 1], value)",
              "if (!matches(view)) return FindRaw(view, pair, counts)",
              "return NormalizeImpl(intersections, result, nullptr)",
              "return NormalizeImpl(intersections, result, &index)"):
    require(translated, token, translated_path)
require(candidate, "const sct::SortedIntersections sorted_intersections(state.candidate_discovery)", CANDIDATE)
if candidate.index("const sct::SortedIntersections sorted_intersections") < candidate.index("ValidatePreparedIntersections(", candidate.index("state.candidate_discovery.DiscoverMasked(")):
    raise RuntimeError("Sorted borrow precedes completed prepared-intersection policy")
require(candidate, "intersections, &value, sorted_intersections)", CANDIDATE)
require(candidate, "&validated_count}, sorted_intersections)", CANDIDATE)
require(values, "ValidateCandidatePublicationsImpl(input, nullptr)", translated_path.with_name("Values.cpp"))
require(values, "ValidateCandidatePublicationsImpl(input, &index)", translated_path.with_name("Values.cpp"))

print("fixed self-contact transaction source proof: PASS")

# Optional numerical facet filters preserve one shared scalar/error fold and
# stream affine spans around the existing serial nonlinear budget decisions.
filter_adapter = (ROOT / "lib_src/collision/self_contact_transaction/FacetFilters.cpp").read_text()
filter_accepted = (ROOT / "lib_src/collision/self_contact_transaction/FacetFilterAccepted.cpp").read_text()
filter_values = (ROOT / "lib_src/collision/self_contact_transaction/FacetFilterValues.cpp").read_text()
filter_header = (ROOT / "lib_src/collision/self_contact_transaction/FacetFilters.h").read_text()
require(types, "bool enable_cuda_facet_filters = false", TYPES)
assert transaction.count("->AcceptedScene(") == 1
assert transaction.count("->AcceptedPairs(") == 2
assert candidate.count("->CandidateScene(") == 1
assert candidate.count("->BeginCandidateChunk(") == 1
assert candidate.count("sct::OptionalFacetPrism(") == 1
assert filter_adapter.index("filters::CompatibleHostArithmetic()") < filter_adapter.index("batch_.Initialize(")
assert "void FacetFilters::Discard()" in filter_adapter and "batch_.DiscardScene()" in filter_adapter
assert "base_input_ = next_input_ = nullptr" in filter_adapter
assert "view.scene_generation != scene_generation_" in filter_adapter + filter_accepted
assert "view.count != prefix" in filter_accepted and "prefix < *count" in filter_accepted
assert "detail::FilterAcceptedFacetPairsWith" in filter_accepted
assert "ClassifyCandidatePairMotion" in filter_values and "PairMotionAction::LinearNodalV1)" in filter_values
assert "CertifyQuadratic" not in filter_values + filter_adapter
for token in ("facet_filters.owned_host_bytes", "facet_filters.device_bytes",
              "facet_filters.startup_host_bytes", "if (config.enable_cuda_facet_filters)"):
    assert token in layout
assert "facet_filters->OutputDisjoint" in transaction and "facet_filters->Discard()" in transaction
assert "if (!reply.supplied) return scalar()" in filter_header
assert "reply.report.status != self_contact_filters::Status::Ok) return false" in filter_header
for source_name in ("FacetFilters.h", "FacetFilterValues.cpp", "FacetFilters.cpp", "FacetFilterAccepted.cpp", "AcceptedFacetFiltering.h"):
    content=(ROOT / "lib_src/collision/self_contact_transaction" / source_name).read_text()
    for forbidden in ("std::vector", "std::map", "std::function", "cudaMalloc", "thread_local"):
        assert forbidden not in content,(source_name,forbidden)
for test_name in ("FacetFilterValueTest.cpp", "FacetFilterAdapterCudaCases.h",
                  "FacetFilterTransactionCudaCases.h", "FacetFilterCudaProbe.cpp", "FacetFilterCudaProbe.h"):
    assert test_name in QUAL_CMAKE.read_text() and test_name in QUAL_BAZEL.read_text(),test_name
print("PASS optional facet-filter source/lifetime/streaming dependency boundary")

assert "device_failure_ = report" in filter_header
for entry in ("Report FacetFilters::Initialize", "Report FacetFilters::PrepareScene", "FacetPrismReply FacetFilters::PrismAt"):
    begin = filter_adapter.index(entry)
    tail = filter_adapter[begin:]
    assert tail.index("device_failure_.status == filters::Status::DeviceFailure") < tail.index("CompatibleHostArithmetic()")
query = filter_adapter[filter_adapter.index("FacetPrismReply FacetFilters::PrismAt"):]
assert query.index("ordinal >= chunk_count_") < query.index("CompatibleHostArithmetic()")
assert "ObserveFailure(report); Discard()" in filter_accepted
