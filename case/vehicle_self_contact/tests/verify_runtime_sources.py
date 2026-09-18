#!/usr/bin/env python3
"""Light source/build proof for the fixed wall+self runtime composition."""

from pathlib import Path
import argparse
import hashlib


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def ordered(text: str, names: list[str], context: str) -> None:
    positions = [text.find(name) for name in names]
    require(all(value >= 0 for value in positions),
            f"{context}: missing ordered stage {names}")
    require(positions == sorted(positions),
            f"{context}: stages are out of order {names}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("app_root", type=Path)
    parser.add_argument("tl_root", type=Path)
    args = parser.parse_args()

    case = args.app_root / "case"
    dynamics = case / "vehicle_dynamics"
    contact = case / "vehicle_self_contact"
    wall = case / "vehicle_wall"

    trial = (dynamics / "Trial.cpp").read_text()
    ordered(
        trial,
        ["s.solids.AssembleAccepted", "wall->Assemble",
         "self_contact->Assemble", "activity->UploadAttempt",
         "s.owner.SealAssembly", "AdvanceStaggeredCin"],
        "accepted transaction")
    ordered(
        trial,
        ["s.solids.EvaluateCandidate", "s.publication.PreparePhysical",
         "wall->Evaluate", "self_contact->SealCandidate"],
        "candidate transaction")

    commit = (dynamics / "VehiclePhysicalDynamics.cpp").read_text()
    receipt_slots = (
        dynamics / "ScratchReceiptRoster.h").read_text()
    require(commit.count("SealPhysicalScratchParticipation(") == 1,
            "dynamics must seal the merged scratch roster exactly once")
    require("ComposeScratchReceipts(" in commit and
            "return {wall.mapped_wall, self_contact.self_contact}" in
            receipt_slots,
            "common commit must preserve mapped-wall then self-contact slots")
    ordered(commit, ["ComposeScratchReceipts(",
                     "SealPhysicalScratchParticipation(",
                     "CommitPhysical("], "common commit")
    ordered(
        commit,
        ["if(wall) wall->Discard()",
         "if(self_contact) self_contact->Discard()",
         "state().owner.Discard()", "state().publication.DiscardTrial()"],
        "common discard")
    require(
        commit.find("// No allocation, device call, readback or other "
                    "fallible work after success.") >
        commit.find("CommitPhysical("),
        "owner commit must retain the no-fallible-work boundary")

    prepare = (contact / "runtime" / "Prepare.cpp").read_text()
    combined = prepare[prepare.find("LoadedWallSelfContact::Prepare("):]
    require(combined.count(
        "ConfigurePhysicalScratchParticipation(") == 1,
        "combined factory must configure the fixed roster exactly once")
    ordered(
        combined,
        ["VehicleWallStartup::PrepareUnconfigured",
         "VehicleSelfContactStartup::PrepareUnconfigured",
         "{wall_entry, self_entry}"],
        "combined startup")
    budget = (contact / "RuntimeBudget.cpp").read_text()
    require("wall_without_publication" in budget and
            "transaction_without_publication" in budget and
            "participation.publication_host_bytes" in budget and
            "ComposeCombinedBudget(" in prepare,
            "combined forecast must replace standalone publication charges")

    startup_header = (
        contact / "VehicleSelfContactStartup.h").read_text()
    private = startup_header.find("private:")
    require(private >= 0 and
            startup_header.find("roster_entry()", private) >= 0 and
            startup_header.find("roster_entry()") >= private,
            "self roster issuer must remain private")
    require("FirstProfileStiffnessPerAreaNPerM3 = 2e9" in
            startup_header,
            "first profile stiffness must remain fixed at 2e9 N/m3")
    require("config.event_capacity <= transaction.max_global_events" in
            (contact / "VehicleSelfContactStartup.cpp").read_text(),
            "force capacity must fit the complete global event ledger")
    setup = (contact / "SelectedSelfContactSource.h").read_text()
    require("source_fields" not in
            (contact / "VehicleSelfContactStartup.cpp").read_text(),
            "runtime startup must not apply original contact fields")
    require("RuntimeCoefficientCensus runtime_coefficients" in setup,
            "setup must retain explicit zero applied-coefficient census")

    wall_stage = (wall / "loaded" / "Stages.cpp").read_text()
    require("SealPhysicalScratchParticipation(" not in wall_stage,
            "wall stage must contribute a receipt, not seal independently")

    cmake = (
        contact / "VehicleSelfContactRuntime.cmake").read_text()
    require("SelfContactTransaction.cmake" in cmake and
            "tl_self_contact_transaction" in cmake,
            "app CMake must own the TL transaction target")
    tl_cmake = (
        args.tl_root / "lib_src" / "collision" /
        "SelfContactTransaction.cmake").read_text()
    require("SelfContactPhysicalActivity.cmake" in tl_cmake and
            "tl_self_contact_physical_activity" in tl_cmake,
            "TL CMake transaction must include physical activity")
    bazel = (
        args.tl_root / "lib_src" / "collision" /
        "BUILD.bazel").read_text()
    transaction = bazel[bazel.find('name = "self_contact_transaction"'):]
    require('":self_contact_physical_activity"' in transaction,
            "TL Bazel transaction must depend on physical activity")
    transaction_cuda = (
        args.tl_root / "lib_utest" / "qualification" /
        "self_contact_transaction" / "CudaTest.cu").read_text()
    for control in [
            "SingleParentZeroPairZeroEventStillParticipatesAndCommits",
            "AcceptedInteriorEeForceCandidateRetryAndRollbackKeepForceSti",
            "ActualT3RemovalFiltersCandidateAndLongInactiveRetryCommits",
            "ExactAffineClosedVfTransitionIsBitwiseDeterministic",
            "ExactFallbackRejectsNonlocalIntersectionAndRollsBackExactly"]:
        require(control in transaction_cuda,
                f"qualified TL transaction control is missing: {control}")
    require("ParticipationFailure" in transaction_cuda and
            "const auto expired = accepted" in transaction_cuda and
            "forged.t3.attempt" in transaction_cuda,
            "missing/stale/late transaction rollback controls are absent")
    active_area = (
        args.tl_root / "lib_utest" / "qualification" /
        "self_contact_active_uses" / "AreaTest.cpp").read_text()
    active_classification = (
        args.tl_root / "lib_utest" / "qualification" /
        "self_contact_active_uses" / "ClassificationTest.cpp").read_text()
    for control in [
            "EdgePointAreaInterpolatesExactlySymmetricallyAndConverges",
            "SymmetricDirectedVertexAndEdgePointDualReferenceV2"]:
        require(control in active_area,
                f"synthetic active-use mechanics coupon is missing {control}")
    require("StandaloneEeAreaRequiresAuthenticatedClosestPointCase" in
            active_classification and
            "BoundaryVertexEdgeMinimum" in active_classification,
            "synthetic strict/boundary active-use policy proof is absent")

    error = (contact / "SelfContactStageError.h").read_text()
    require("SelfContactTransactionReport report_" in error and
            "required_events() const noexcept" in error and
            "report.candidate > configured_event_capacity" in error,
            "app stage error must preserve the typed exact event count")
    gate = (
        case / "vehicle_startup" / "shell_execution" / "tests" /
        "self_contact" / "RuntimeGateTest.cpp").read_text()
    for value in ["376930", "337092", "315963", "653055",
                  "1584464", "5989248", "4096"]:
        require(value in gate,
                f"actual runtime gate is missing exact value {value}")
    require("1542100696" in gate or "3526100688" in gate,
            "actual runtime gate is missing its exact arena value")
    require("SelfContactTransactionLimits::Vehicle(" in gate and
            "V5_SELF_CONTACT_RUNTIME" in gate and
            "SelfContactOnly::Preflight(" in gate and
            "SelfContactOnly::Prepare(" in gate and
            "LoadedWallSelfContact::Prepare(" in gate,
            "actual self-only/wall+self runtime gate is incomplete")
    require("IGNORE=1" not in gate,
            "runtime gate must not bypass any transaction result")
    fixture_cmake = (
        case / "vehicle_startup" / "shell_execution" /
        "SelfContactTests.cmake").read_text()
    for name in [
            "vehicle_self_contact_acceptance_v5",
            "vehicle_wall_self_contact_acceptance_v5",
            "FullV5SingleAttemptSealsAndDiscards",
            "FullV5CombinedSingleAttemptSealsBothReceipts"]:
        require(name in fixture_cmake,
                f"runtime CTest registration is missing: {name}")
    require("RESOURCE_LOCK vehicle_self_contact_gpu" in fixture_cmake and
            "TIMEOUT 7200" in fixture_cmake and
            'LABELS "acceptance-v5;large;' in fixture_cmake and
            "RuntimeGateTest.cpp" in fixture_cmake,
            "runtime gates need source proof and finite serialized properties")
    candidate_coupon = (
        case / "vehicle_startup" / "shell_execution" / "tests" /
        "self_contact" / "CandidateRigidCouponTest.cpp").read_text()
    ambiguous_roster = (
        case / "vehicle_startup" / "shell_execution" / "tests" /
        "self_contact" / "NonlinearAmbiguousRoster.inc").read_text()
    nonlinear_fixture_dir = (
        case / "vehicle_startup" / "shell_execution" / "tests" /
        "self_contact")
    # Qualification helpers are shared by the original coupons and the actual
    # second-interval capture; they no longer live in one giant test unit.
    for helper in ("CandidateCapture.cpp", "CoverageFixtureCapture.cpp",
                   "CoverageFixtureGeometry.cpp", "CoverageFixtureInspection.cpp"):
        candidate_coupon += "\n" + (nonlinear_fixture_dir / helper).read_text()
    nonlinear_fixture_header = (
        nonlinear_fixture_dir / "NonlinearCoverageFixture.h").read_text()
    nonlinear_fixture_test = (
        nonlinear_fixture_dir /
        "NonlinearCoverageFixtureTest.cpp").read_text()
    nonlinear_fixture = (
        nonlinear_fixture_dir /
        "NonlinearAmbiguousFixture.bin").read_bytes()
    require(ambiguous_roster.count("},") == 826,
            "nonlinear ambiguous roster must pin exactly 826 pairs")
    require(len(nonlinear_fixture) == 12043248,
            "nonlinear fixture size changed")
    require(hashlib.sha256(nonlinear_fixture).hexdigest() ==
            "9f9cc331d07ee0dd4b12a699ecda73d00a3333fdc5788aa38e4fa79415e04545",
            "nonlinear fixture SHA-256 changed")
    for token in [
            "ROBO_COVERAGE_EXPECTED_PAIRS 826",
            "ExpectedRosterDigest",
            "ExpectedSchemaHash",
            "ExpectedSourceHash",
            "ExpectedProfileHash",
            "ExpectedDtHash",
            "ExpectedPayloadHash",
            "ExpectedPolicyResultDigest",
            "MaximumBytes = 64u << 20",
            "FixedTriangleFeatureCandidate",
            "AcceptedEventCertificate",
            "AcceptedFeaturePolicyEvidence",
            "PhaseIdentity",
            "FacetQuadraticCoefficients",
            "representation_error_m",
            "RosterDigest(",
            "SourceHash("]:
        require(token in nonlinear_fixture_header,
                f"nonlinear fixture schema is missing {token}")
    linear_fixture_header = (
        nonlinear_fixture_dir /
        "LinearCoverageFixture.h").read_text()
    linear_fixture_test = (
        nonlinear_fixture_dir /
        "LinearCoverageFixtureTest.cpp").read_text()
    linear_fixture = (
        nonlinear_fixture_dir /
        "LinearWorkExhaustedFixture.bin").read_bytes()
    require(len(linear_fixture) == 21408,
            "linear WorkExhausted fixture size changed")
    require(hashlib.sha256(linear_fixture).hexdigest() ==
            "4ffca5180edd06640c64ca7a8ec2c84b9b460dda174f845984d77e4d0ae15236",
            "linear WorkExhausted fixture SHA-256 changed")
    for token in [
            "ExpectedPairs", "ExpectedRosterDigest",
            "ExpectedCensusDigest", "ExpectedSchemaHash",
            "ExpectedSourceHash", "ExpectedProfileHash",
            "ExpectedDtHash", "ExpectedPayloadHash",
            "ExpectedPayloadBytes", "ExpectedPolicyResultDigest",
            "enforce_pins"]:
        require(
            token in linear_fixture_header +
                nonlinear_fixture_header,
            f"linear fixture schema is missing {token}")
    for token in [
            "CompleteRosterHasNoUnexplainedWorkExhaustion",
            "WorkExhausted", "CertifyQuadraticFacetPolicyCoverage",
            "unexplained", "4095", "std::memcmp",
            "runtime_s=", "2142381", "2230072",
            "permuted_owners", "no_owner", "one_bit_owner",
            "one_bit_geometry", "has_contact_transition",
            "345915413587096", "std::uint64_t{10067}",
            "SourceHash(data.pairs)"]:
        require(token in linear_fixture_test,
                f"linear fixture replay is missing {token}")
    for token in [
            "FrozenRosterResolvesWithAuthenticatedPolicyEvidence",
            "PartialMixedLinearCandidateIsFrozenExactly",
            "ReceiptCandidatePair = 289078",
            "inconclusive_u=[0,1/1048576]",
            "intersection_time_numerator",
            "VerifyEndpoint(",
            "CertifyQuadraticFacetPolicyCoverage(",
            "AcceptedOwnersFromPolicy(",
            "AcceptedExclusionsFromPolicy(",
            "MissingAcceptedOwner",
            "true_nonexcluded_ownerless=",
            "std::reverse(",
            "runtime_s="]:
        require(token in nonlinear_fixture_test,
                f"nonlinear fixture replay is missing {token}")
    for token in [
            "2100005", "2100048", "PhysicalStepS = 2e-7",
            "EndpointCorrectedSecondOrderDriftV1",
            "BuildRigidMemberSweepBounds(", "0x0da4u",
            "PartialOrMixedRigid", "shared_vertices",
            "accepted_minimum_nonlocal_m",
            "prepared_minimum_nonlocal_m",
            "V5_CANDIDATE_142_RIGID_GROUP",
            "Candidate143MixedFacetHasExactAffineRepresentedMotion",
            "AffineMixedLocalFacet = 1",
            "CertifyRigidFacetAffineMotion(",
            "first.certified_affine = affine[0]",
            "second.certified_affine = affine[1]",
            "endpoint_chord_substitution=0"]:
        require(token in candidate_coupon,
                f"candidate rigid coupon is missing {token}")
    for token in [
            "Candidate2694SourceTopologyAndPhysicalOnlyBaseline",
            "2100124", "2209533",
            "candidate_facet=222",
            "accepted_self_contact_assembly_included=0",
            "BuildRigidFacetQuadraticCoefficients(",
            "CertifyQuadraticFacetSeparation(",
            "V5_CANDIDATE_2694_EXACT_Q_INTERVAL"]:
        require(token in candidate_coupon,
                f"candidate-2694 baseline coupon is missing {token}")
    for token in [
            "ResidualAndPersistentPairsUseAuthenticatedPreparedBits",
            "PrepareAcceptedAssembly",
            "CertifyLinearResidualSeparation(",
            "QualificationAccess::AcceptedCertificates(",
            "V5_LINEAR_PERSISTENT_CERTIFICATE",
            "V5_LINEAR_PERSISTENT_ACCEPTED",
            "V5_LINEAR_PERSISTENT_BITS",
            "accepted_feature_matches",
            "canonical_persistent.status",
            "V5_LINEAR_FAMILY_PATTERN",
            "V5_LINEAR_FAMILY_SUMMARY",
            "V5_LINEAR_FAMILY_UNRESOLVED"]:
        require(token in candidate_coupon,
                f"residual-translation coupon is missing {token}")
    for token in [
            "CompleteAcceptedAssemblyRosterSkipsLinearExactTraversal",
            "ClassifyPreparedCandidateCensus(",
            "V5_NONLINEAR_ROSTER",
            "V5_NONLINEAR_AMBIGUOUS",
            "ExpectedNonlinearRosterDigest",
            "ExpectedAmbiguousRosterDigest",
            "ExpectedLinearCensusDigest",
            "ExpectedLinearFixtureRosterDigest",
            "ExpectedAmbiguousRoster",
            'include "NonlinearAmbiguousRoster.inc"',
            "15183149279991149367",
            "CertifyQuadraticResidualSeparation(",
            "CertifyPersistentQuadraticContact(",
            "after_classes",
            "std::atomic<std::size_t>",
            "workers.reserve(24)",
            "workers.emplace_back(inspect)",
            "ROBO_NONLINEAR_FIXTURE_OUTPUT",
            "FreezeNonlinearPair(",
            "FreezeAcceptedPolicies(",
            "V5_LINEAR_ROSTER",
            "ROBO_LINEAR_FIXTURE_OUTPUT",
            "ROBO_LINEAR_CENSUS_ONLY",
            "5809241", "3433535", "3433481",
            "6473154677596308446",
            "V5_NONLINEAR_COVERAGE",
            "nonlinear_fixture::Write("]:
        require(token in candidate_coupon,
                f"nonlinear roster coupon is missing {token}")
    require("CandidateRigidCouponTest.cpp" in fixture_cmake and
            "vehicle_self_contact_candidate_rigid_coupon" in fixture_cmake and
            "vehicle_self_contact_nonlinear_roster_coupon" in
            fixture_cmake and
            "vehicle_self_contact_nonlinear_fixture" in
            fixture_cmake and
            "vehicle_self_contact_linear_fixture" in fixture_cmake and
            "robo_dyna_vehicle_self_contact_linear_fixture_check" in
            fixture_cmake and
            "robo_dyna_vehicle_self_contact_nonlinear_fixture_check" in
            fixture_cmake and
            "VehicleSelfContactNonlinearRosterCoupon.*" in fixture_cmake and
            "accepted-assembly;nonlinear-roster" in fixture_cmake and
            "vehicle_self_contact_residual_translation_coupon" in
            fixture_cmake and
            "VehicleSelfContactAcceptedAssemblyCoupon.*" in fixture_cmake and
            '"coupon;real-geometry;v5-candidate"' in fixture_cmake,
            "candidate-142 coupon registration is incomplete")
    tl_candidate = (
        args.tl_root / "lib_src" / "collision" /
        "self_contact_transaction" / "Candidate.cpp").read_text()
    tl_values = (
        args.tl_root / "lib_src" / "collision" /
        "self_contact_transaction" / "Values.cpp").read_text()
    tl_arena = (
        args.tl_root / "lib_src" / "collision" /
        "self_contact_transaction" / "Arena.cpp").read_text()
    tl_rigid_sweep = (
        args.tl_root / "lib_src" / "collision" /
        "self_contact_transaction" / "RigidSweep.cpp").read_text()
    tl_local = (
        args.tl_root / "lib_src" / "collision" /
        "self_contact_transaction" / "LocalContact.cpp").read_text()
    tl_translated_local = (
        args.tl_root / "lib_src" / "collision" /
        "self_contact_transaction" / "TranslatedLocal.cpp").read_text()
    ordered(tl_local, [
        "BuildFixedTriangleFeatureTaskMask(",
        "EvaluatePairFeaturesMaskedOnce(",
        "CertifyQuadraticUnmaskedSeparation(",
        "LinearResidualSeparationStatus::CertifiedSeparated",
        "return CertifyQuadraticLocalTopology("],
        "local contact whole-interval thickness/topology proof")
    topology = tl_rigid_sweep[
        tl_rigid_sweep.find("NonlinearSeparationResult CertifyQuadraticLocalTopology("):
        tl_rigid_sweep.find("NonlinearSeparationResult CertifyQuadraticFacetCoverage(")]
    ordered(topology, [
        "endpoint < 2", "ClassifyPairIntersection(",
        "RequiresIntersectionAdmission(intersection)",
        "return CertifyQuadraticFacetCoverageImpl("],
        "local topology endpoint premises and continuous coverage")
    require("max_work, max_depth, true, true" in topology,
            "local topology must require bounded whole-interval geometry")
    candidate_witness = tl_candidate[
        tl_candidate.find("const auto translated_local ="):
        tl_candidate.find("value.work = total_work;")]
    ordered(candidate_witness, [
        "NormalizeExactTranslatedLocal(",
        "translated_local != sct::TranslatedLocalStatus::Certified",
        "RepresentedIntervalClassification::CertifiedCrossingContact",
        "RepresentedIntervalReason::WorkExhausted",
        "CertifyQuadraticFacetPolicyCoverage(",
        "NonlinearSeparationStatus::CertifiedLocalIntersection",
        "RepresentedIntersectionGeometry::CertifiedLocalTopology"],
        "candidate first witness requires complete continuous policy")
    ordered(tl_translated_local, [
        "HasExactCommonTranslationProof(result->geometry)",
        "RequiresIntersectionAdmission(*intersection)",
        "result->geometry = RepresentedIntersectionGeometry::CertifiedLocalTopology"],
        "translated local normalization requires the native invariant-path proof")
    local_start = tl_values.find(
        "if (LocallyExcluded(input.intersections, crossing.key)) {")
    local_publication = tl_values[
        local_start:tl_values.find("bool edge_edge = false;", local_start)]
    ordered(local_publication, [
        "if (crossing.geometry !=",
        "RepresentedIntersectionGeometry::CertifiedLocalTopology)",
        "return Failure(SelfContactTransactionStatus::CandidateRejected",
        "Endpoint-local intersection lacks continuous topology proof",
        "SelfContactCandidateDisposition::ExcludedLocalIntersection"],
        "local publication rejects an endpoint-only intersection witness")
    require("raw_crossings.count != crossing_pair_count" in tl_candidate and
            "S::UnsupportedMotion" in tl_candidate and
            "RepresentedIntervalReason::UnsupportedMotion" in tl_candidate and
            "input.crossings.count != input.pair_count" in tl_values and
            "return Failure(SelfContactTransactionStatus::UnresolvedCandidate" in tl_values,
            "TL must retain complete pair accounting and reject uncertified motion")
    require("first.certified_affine && second.certified_affine" in tl_arena and
            "CertifyRigidPointAffineMotion(" in tl_rigid_sweep and
            "represented_q[component]" in tl_rigid_sweep,
            "TL does not exactly certify represented affine rigid motion")
    require("CertifyQuadraticFacetSeparation(" in tl_rigid_sweep and
            "SubdivideSeparation(" in tl_rigid_sweep and
            "CertifyQuadraticFacetCoverage(" in tl_rigid_sweep and
            "CertifyQuadraticFacetPolicyCoverage(" in tl_rigid_sweep and
            "SubdivideCoverage(" in tl_rigid_sweep and
            "MissingAcceptedOwner" in tl_rigid_sweep and
            "nonlinear_subdivision_work_per_pair" in tl_candidate,
            "TL does not bound nonlinear represented separation")
    real_geometry = (
        case / "vehicle_startup" / "shell_execution" / "tests" /
        "self_contact" / "RealGeometryTest.cpp").read_text()
    for token in [
            "2100084", "2279821", "2288735",
            "2125365", "2348922", "2352112", "0x6c30",
            "2382006", "2382159", "2402419", "2402383", "0x775b",
            "FixedContactFacet", "FixedTriangleFeatureDiscovery",
            "DiscoverMasked", "TriangleIntersection",
            "DiscoveryHash", "zero_nonparallel_ee",
            "StrictInteriorInteriorMinimum",
            "BoundaryVertexEdgeMinimum",
            "FirstAcceptedIntersectionIsExactSharedVertexOnly",
            "FullV5SecondAcceptedIntersectionIsBoundaryToParentDiagonal",
            "SharedEdgeOnly",
            "AdjacentFacetSeamsCanonicalizeWithoutDroppingParentLocalOwners",
            "c82f1886b8935d69ff7db4c29c700370e3a057579fab80d02664a253bc7af1c8",
            "a96bc12b9c8467253da0898565c7875ad80f58f963b45d1dc405f5dddab76b1d"]:
        require(token in real_geometry,
                f"authenticated real-geometry coupon is missing {token}")
    require("VehiclePhysicalDynamics" not in real_geometry and
            ".78927" not in real_geometry and
            ".340809" not in real_geometry and
            ".679739" not in real_geometry,
            "real geometry coupon must not load dynamics or copy diagnostic parameters")
    census_values = (contact / "InitialCensusValues.cpp").read_text()
    census_header = (
        contact / "VehicleSelfContactInitialCensus.h").read_text()
    census_source = (
        contact / "VehicleSelfContactInitialCensus.cpp").read_text()
    sample_header = (
        contact / "InitialFeatureSampleValues.h").read_text()
    sample_source = (
        contact / "InitialFeatureSampleValues.cpp").read_text()
    values_cmake = (
        contact / "VehicleSelfContactValues.cmake").read_text()
    tl_filter_header = (
        args.tl_root / "lib_src" / "collision" /
        "SelfContactFilterCertificates.h").read_text()
    tl_filter_source = (
        args.tl_root / "lib_src" / "collision" /
        "SelfContactFilterCertificates.cpp").read_text()
    tl_transaction_source = (
        args.tl_root / "lib_src" / "collision" /
        "self_contact_transaction" / "Source.cpp").read_text()
    for token in [
            "excluded_same_rigid_group", "coordinate_aabb_separated",
            "face_axis_separated", "edge_cross_axis_separated",
            "vertex_edge_axis_separated",
            "vertex_vertex_axis_separated",
            "exact_remaining", "exact_sample_count",
            "exact_sample_hash", "category_hash",
            "geometry_evaluation_us", "filter_census_us",
            "rerun_filter_hash"]:
        require(token in census_header + census_values,
                f"actual V5 filter census is missing {token}")
    for token in [
            "potential_tasks", "local_masked_tasks",
            "exact_executed_tasks", "raw_feature_candidates",
            "feature_candidates", "raw_intersections", "intersections",
            "nonlocal_intersections", "first_nonlocal_intersection",
            "RequiresIntersectionAdmission",
            "feature_hash", "intersection_hash", "discovery_us",
            "exact_tasks_per_second"]:
        require(token in sample_header + sample_source,
                f"actual V5 exact sample is missing {token}")
    require("ClassifyAcceptedFacetPair(" in census_values and
            "ClassifyAcceptedFacetPair(" in tl_filter_header and
            "ClassifyAcceptedFacetPair(" in tl_filter_source and
            "ClassifyAcceptedFacetPair(" in tl_transaction_source,
            "app and transaction must share the production filter certificate")
    require("RequiresIntersectionAdmission(" in tl_transaction_source and
            "Accepted nonlocal triangle intersection is rejected" in
            tl_transaction_source,
            "current TL nonlocal-intersection rejection policy is missing")
    require("BuildFixedTriangleFeatureTaskMask(" in sample_source and
            ".DiscoverMasked(" in sample_source and
            "InitialExactFeatureSampleCapacity = 262144" in sample_header and
            "InitialExactFeatureChunkCapacity = 4096" in sample_header and
            "InitialExactFeatureChunkCapacity, 4" in census_source and
            "InitialExactFeatureRerunThresholdUs" in census_source and
            "InitialFeatureSampleValues.cpp" in values_cmake,
            "actual exact sample must use bounded shared-mask discovery")
    require("RepresentedIntervalCrossing" not in census_source + sample_source and
            "SelfContactForce" not in sample_source,
            "actual exact sample must stop before crossing and dynamics")
    require('LABELS "coupon;real-geometry;v5-exact-sample"' in
            fixture_cmake and
            "TIMEOUT 180" in fixture_cmake,
            "actual filter census needs its bounded coupon registration")
    real_cmake = (contact / "CMakeLists.txt").read_text()
    require("vehicle_self_contact_real_geometry_coupon" in real_cmake and
            '"RealYarisSelfContactGeometry.*"' in real_cmake and
            "RUN_SERIAL TRUE PROCESSORS 1 TIMEOUT 30" in real_cmake and
            'LABELS "coupon;real-geometry"' in real_cmake and
            "FixedTriangleFeatureDiscovery.cmake" in real_cmake and
            "ROBO_DYNA_VEHICLE_SELF_CONTACT_REAL_GEOMETRY" in real_cmake,
            "bounded authenticated real-geometry CTest registration is absent")
    acceptance_guard = fixture_cmake.find(
        "if(ROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE)")
    self_acceptance = fixture_cmake.find(
        "add_test(NAME vehicle_self_contact_acceptance_v5",
        acceptance_guard)
    wall_guard = fixture_cmake.find(
        'if(EXISTS "${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")',
        self_acceptance)
    wall_acceptance = fixture_cmake.find(
        "add_test(NAME vehicle_wall_self_contact_acceptance_v5",
        wall_guard)
    require(acceptance_guard >= 0 and
            acceptance_guard < self_acceptance <
            wall_guard < wall_acceptance,
            "full V5 runtime registration is not acceptance/manifest guarded")
    root_cmake = (args.app_root / "CMakeLists.txt").read_text()
    shell_cmake = (
        case / "vehicle_startup" / "shell_execution" /
        "CMakeLists.txt").read_text()
    for text in (root_cmake, shell_cmake):
        require("option(ROBO_DYNA_ENABLE_V5_SELF_CONTACT_ACCEPTANCE" in text,
                "V5 acceptance cache option is missing")
        require(
            '"Register full canonical V5 self-contact runtime acceptance tests" OFF)'
            in text,
            "V5 acceptance cache option must default OFF")

    runtime_values = (contact / "tests" / "RuntimeValuesTest.cpp").read_text()
    for token in ("ComposeCombinedBudget(",
                  "ComposeScratchReceipts(",
                  "VehicleSelfContactRuntimeCoupon",
                  "2 * sizeof(tl::fea::ShellPhysicalScratchParticipation)"):
        require(token in runtime_values,
                f"small app runtime coupon is missing {token}")
    contact_cmake = (contact / "CMakeLists.txt").read_text()
    require('LABELS "unit"' in contact_cmake and
            'LABELS "coupon"' in contact_cmake,
            "small app runtime tiers need unit/coupon labels")

    print("vehicle self-contact runtime source/CMake/Bazel proof passed")


if __name__ == "__main__":
    main()
