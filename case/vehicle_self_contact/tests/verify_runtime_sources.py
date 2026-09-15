#!/usr/bin/env python3
"""Light source/build proof for the fixed wall+self runtime composition."""

from pathlib import Path
import argparse


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
            "ExactPassThroughUnresolvedReasonRollsBackAndRetriesExactly"]:
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
            "vehicle_self_contact_runtime_${runtime_gate}",
            "vehicle_wall_self_contact_runtime_${runtime_gate}",
            "FullV5ForecastStartupOwnsExactIdentityAndMemory",
            "FullV5OneAttemptIsTypedFailClosedAndRetryStable",
            "FullV5CombinedForecastStartupOwnsBothFixedSlotsOnce",
            "FullV5CombinedAttemptSealsBothReceiptsAndRetries"]:
        require(name in fixture_cmake,
                f"runtime CTest registration is missing: {name}")
    require("RESOURCE_LOCK vehicle_self_contact_gpu" in fixture_cmake and
            "TIMEOUT ${runtime_timeout}" in fixture_cmake and
            'LABELS "acceptance-v5;' in fixture_cmake and
            "RuntimeGateTest.cpp" in fixture_cmake,
            "runtime gates need source proof and finite serialized properties")
    candidate_coupon = (
        case / "vehicle_startup" / "shell_execution" / "tests" /
        "self_contact" / "CandidateRigidCouponTest.cpp").read_text()
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
            "V5_LINEAR_PERSISTENT_BITS"]:
        require(token in candidate_coupon,
                f"residual-translation coupon is missing {token}")
    require("CandidateRigidCouponTest.cpp" in fixture_cmake and
            "vehicle_self_contact_candidate_rigid_coupon" in fixture_cmake and
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
    require("LocallyExcluded(" in tl_candidate and
            "crossing_pair_count" in tl_candidate and
            "RepresentedIntervalReason::UnsupportedMotion" in tl_values,
            "TL does not apply exact local incidence before unsupported motion")
    require("first.certified_affine && second.certified_affine" in tl_arena and
            "CertifyRigidPointAffineMotion(" in tl_rigid_sweep and
            "represented_q[component]" in tl_rigid_sweep,
            "TL does not exactly certify represented affine rigid motion")
    require("CertifyQuadraticFacetSeparation(" in tl_rigid_sweep and
            "SubdivideSeparation(" in tl_rigid_sweep and
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
    self_loop = fixture_cmake.find(
        "foreach(runtime_gate IN ITEMS startup one_attempt)",
        acceptance_guard)
    wall_guard = fixture_cmake.find(
        'if(EXISTS "${ROBO_DYNA_VEHICLE_WALL_MANIFEST}")',
        self_loop)
    wall_loop = fixture_cmake.find(
        "foreach(runtime_gate IN ITEMS startup one_attempt)",
        wall_guard)
    require(acceptance_guard >= 0 and
            acceptance_guard < self_loop < wall_guard < wall_loop,
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
