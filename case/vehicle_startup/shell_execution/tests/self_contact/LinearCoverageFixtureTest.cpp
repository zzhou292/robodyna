#include "LinearCoverageFixture.h"

#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/RepresentedIntervalCrossing.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <numeric>
#include <vector>

#ifndef ROBO_LINEAR_FIXTURE_PATH
#error "ROBO_LINEAR_FIXTURE_PATH is required"
#endif

namespace crash::cases::vehicle_startup::shell_execution::
    self_contact_test {
namespace {

namespace contact = tlfea::contact;
namespace fixture = linear_fixture;
namespace sct = tlfea::contact::self_contact_transaction;

contact::RepresentedTrianglePath Path(
    const contact::CurrentFixedTriangle& accepted,
    const contact::CurrentFixedTriangle& prepared) {
    contact::RepresentedTrianglePath result;
    result.key = {
        prepared.key.source_instance_id, prepared.key.parent_eid,
        prepared.key.level, prepared.key.local_facet};
    result.motion = contact::RepresentedMotion::LinearNodalV1;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
        result.vertices[vertex].key =
            prepared.vertex_keys[vertex];
        result.vertices[vertex].endpoint[0] =
            accepted.vertices[vertex];
        result.vertices[vertex].endpoint[1] =
            prepared.vertices[vertex];
        result.edge_keys[vertex] = prepared.edge_keys[vertex];
    }
    return result;
}

std::vector<sct::AcceptedFeatureExclusionCertificate>
Exclusions(const fixture::Pair& pair) {
    std::vector<sct::AcceptedFeatureExclusionCertificate> result;
    for (std::size_t feature = 0;
         feature < pair.accepted_features.size(); ++feature) {
        const auto& evidence = pair.accepted_policy[feature];
        if (evidence.pair_status !=
                contact::SelfContactPairStatus::
                    ExcludedSameRigidGroup ||
            evidence.endpoint_support[0].status !=
                contact::SelfContactSupportStatus::
                    CompleteRigidGroup ||
            evidence.endpoint_support[1].status !=
                contact::SelfContactSupportStatus::
                    CompleteRigidGroup ||
            evidence.endpoint_support[0].complete_rigid_group !=
                evidence.endpoint_support[1].
                    complete_rigid_group)
            continue;
        result.push_back({
            pair.accepted_features[feature],
            static_cast<std::uint32_t>(
                evidence.endpoint_support[0].
                    complete_rigid_group)});
    }
    return result;
}

void VerifyEndpoint(
    const contact::CurrentFixedTriangle triangles[2],
    std::uint16_t expected_mask,
    const std::vector<
        contact::FixedTriangleFeatureCandidate>& expected,
    const std::vector<
        contact::FixedTriangleIntersection>& intersections) {
    contact::FixedTriangleFeatureTaskMask mask;
    ASSERT_EQ(
        contact::BuildFixedTriangleFeatureTaskMask(
            triangles[0], triangles[1], &mask),
        contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(mask.local_tasks, expected_mask);
    std::array<contact::FixedTriangleFeatureCandidate, 15> observed;
    contact::fixed_triangle_features::PairFeatureResult result;
    ASSERT_EQ(
        contact::fixed_triangle_features::
            EvaluatePairFeaturesMaskedOnce(
                triangles[0], triangles[1], mask,
                observed.data(), observed.size(), &result),
        contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(result.feature_count, expected.size());
    ASSERT_EQ(
        std::memcmp(
            observed.data(), expected.data(),
            expected.size() * sizeof(expected[0])),
        0);
    contact::FixedTriangleIntersection intersection;
    bool intersects = false;
    ASSERT_EQ(
        contact::fixed_triangle_features::
            ClassifyPairIntersection(
                triangles[0], triangles[1],
                &intersection, &intersects),
        contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(
        static_cast<std::size_t>(intersects),
        intersections.size());
    if (intersects)
        EXPECT_EQ(
            std::memcmp(
                &intersection, intersections.data(),
                sizeof(intersection)),
            0);
}

TEST(VehicleSelfContactLinearFixture,
     CompleteRosterHasNoUnexplainedWorkExhaustion) {
    const auto started = std::chrono::steady_clock::now();
    const auto data = fixture::Read(ROBO_LINEAR_FIXTURE_PATH);
    const auto source_before =
        fixture::SourceHash(data.pairs);
    ASSERT_EQ(data.schema_hash, nonlinear_fixture::SchemaHash());
    ASSERT_EQ(data.pairs.size(), fixture::ExpectedPairs);
    ASSERT_EQ(data.roster_digest, fixture::ExpectedRosterDigest);
    ASSERT_EQ(
        data.nonlinear_roster_digest,
        fixture::ExpectedCensusDigest);
    ASSERT_EQ(data.source_hash, fixture::ExpectedSourceHash);
    ASSERT_EQ(data.profile_hash, fixture::ExpectedProfileHash);
    ASSERT_EQ(data.dt_hash, fixture::ExpectedDtHash);
    ASSERT_EQ(data.payload_hash, fixture::ExpectedPayloadHash);
    ASSERT_EQ(data.payload_bytes, fixture::ExpectedPayloadBytes);

    contact::RepresentedIntervalCrossing crossing;
    contact::RepresentedIntervalLimits limits;
    limits.max_paths = 2;
    limits.max_input_pairs = 1;
    limits.max_results = 1;
    limits.max_work_per_pair = 4095;
    limits.max_total_work = 4095;
    limits.max_depth = 20;
    ASSERT_EQ(
        crossing.Initialize(limits).status,
        contact::RepresentedIntervalStatus::Ok);

    std::array<std::size_t, 10> after{};
    std::size_t unexplained = 0;
    std::size_t baseline_work = 0;
    std::size_t policy_work = 0;
    std::uint64_t digest = 1469598103934665603ull;
    for (std::size_t index = 0;
         index < data.pairs.size(); ++index) {
        const auto& pair = data.pairs[index];
        EXPECT_EQ(pair.prepared[0].key.parent_eid, 2142381u);
        EXPECT_EQ(pair.prepared[0].key.local_facet, 1u);
        EXPECT_EQ(pair.prepared[1].key.parent_eid, 2230072u);
        EXPECT_EQ(pair.prepared[1].key.local_facet, 1u);
        EXPECT_EQ(pair.accepted_mask, 0u);
        EXPECT_EQ(pair.prepared_mask, 0u);
        EXPECT_TRUE(pair.accepted_intersections.empty());
        EXPECT_TRUE(pair.prepared_intersections.empty());
        for (const auto& quadratic : pair.quadratic) {
            ASSERT_TRUE(quadratic.complete);
            for (const auto& vertex : quadratic.q)
                for (const auto& component : vertex) {
                    EXPECT_EQ(component.lower, 0);
                    EXPECT_EQ(component.upper, 0);
                }
        }
        VerifyEndpoint(
            pair.accepted, pair.accepted_mask,
            pair.accepted_features,
            pair.accepted_intersections);
        VerifyEndpoint(
            pair.prepared, pair.prepared_mask,
            pair.prepared_features,
            pair.prepared_intersections);
        const contact::RepresentedTrianglePath paths[]{
            Path(pair.accepted[0], pair.prepared[0]),
            Path(pair.accepted[1], pair.prepared[1])};
        const contact::RepresentedTrianglePair input{0, 1};
        ASSERT_EQ(
            crossing.Certify(paths, 2, &input, 1).status,
            contact::RepresentedIntervalStatus::Ok);
        const auto baseline = crossing.results();
        ASSERT_TRUE(baseline.complete);
        ASSERT_EQ(baseline.count, 1u);
        ASSERT_EQ(
            baseline.data[0].classification,
            contact::RepresentedIntervalClassification::Unresolved);
        ASSERT_EQ(
            baseline.data[0].reason,
            contact::RepresentedIntervalReason::WorkExhausted);
        ASSERT_EQ(baseline.data[0].work, 4095u);
        baseline_work += baseline.data[0].work;

        const auto exclusions = Exclusions(pair);
        const auto certify = [&](bool reverse) {
            return reverse
                ? sct::CertifyQuadraticFacetPolicyCoverage(
                      pair.accepted[1], pair.prepared[1],
                      pair.quadratic[1], pair.half_thickness[1],
                      pair.accepted[0], pair.prepared[0],
                      pair.quadratic[0], pair.half_thickness[0],
                      2e-7, pair.accepted_owners.data(),
                      pair.accepted_owners.size(),
                      exclusions.data(), exclusions.size(),
                      4095, 20)
                : sct::CertifyQuadraticFacetPolicyCoverage(
                      pair.accepted[0], pair.prepared[0],
                      pair.quadratic[0], pair.half_thickness[0],
                      pair.accepted[1], pair.prepared[1],
                      pair.quadratic[1], pair.half_thickness[1],
                      2e-7, pair.accepted_owners.data(),
                      pair.accepted_owners.size(),
                      exclusions.data(), exclusions.size(),
                      4095, 20);
        };
        const auto resolved = certify(false);
        const auto repeated = certify(false);
        const auto reversed = certify(true);
        const contact::FixedTriangleFeatureView prepared_features{
            pair.prepared_features.data(),
            pair.prepared_features.size(), true};
        const contact::FixedTriangleIntersectionView
            prepared_intersections{
                pair.prepared_intersections.data(),
                pair.prepared_intersections.size(), true};
        const auto residual =
            sct::CertifyLinearResidualSeparation(
                pair.accepted[0], pair.prepared[0],
                pair.half_thickness[0],
                pair.accepted[1], pair.prepared[1],
                pair.half_thickness[1],
                prepared_features, prepared_intersections);
        const auto persistent =
            sct::CertifyPersistentLinearContact(
                pair.accepted[0], pair.prepared[0],
                pair.half_thickness[0],
                pair.accepted[1], pair.prepared[1],
                pair.half_thickness[1], prepared_features,
                pair.accepted_owners.data(),
                pair.accepted_owners.size());
        EXPECT_FALSE(residual.exact_common_translation);
        EXPECT_EQ(
            residual.status,
            sct::LinearResidualSeparationStatus::PotentialContact);
        EXPECT_EQ(
            persistent.status,
            sct::PersistentLinearContactStatus::PotentialChange);
        EXPECT_EQ(
            resolved.status,
            sct::NonlinearSeparationStatus::PotentialContact);
        EXPECT_EQ(resolved.work, 29u);
        EXPECT_EQ(resolved.deepest, 20u);
        EXPECT_EQ(resolved.covered_cells, 8u);
        EXPECT_EQ(resolved.separated_cells, 0u);
        EXPECT_FALSE(resolved.has_intersection);
        EXPECT_TRUE(resolved.has_unresolved_cell);
        EXPECT_EQ(repeated.status, resolved.status);
        EXPECT_EQ(repeated.work, resolved.work);
        EXPECT_EQ(
            repeated.proof_digest, resolved.proof_digest);
        EXPECT_EQ(reversed.status, resolved.status);
        EXPECT_EQ(reversed.work, resolved.work);
        EXPECT_EQ(reversed.deepest, resolved.deepest);
        EXPECT_EQ(
            reversed.unresolved_path, resolved.unresolved_path);
        EXPECT_EQ(
            reversed.unresolved_depth, resolved.unresolved_depth);
        auto permuted_owners = pair.accepted_owners;
        std::reverse(permuted_owners.begin(), permuted_owners.end());
        const auto permuted =
            sct::CertifyQuadraticFacetPolicyCoverage(
                pair.accepted[0], pair.prepared[0],
                pair.quadratic[0], pair.half_thickness[0],
                pair.accepted[1], pair.prepared[1],
                pair.quadratic[1], pair.half_thickness[1],
                2e-7, permuted_owners.data(),
                permuted_owners.size(),
                exclusions.data(), exclusions.size(),
                4095, 20);
        EXPECT_EQ(permuted.status, resolved.status);
        EXPECT_EQ(permuted.work, resolved.work);
        EXPECT_EQ(permuted.proof_digest, resolved.proof_digest);
        EXPECT_EQ(
            permuted.unresolved_path, resolved.unresolved_path);
        const auto capped =
            sct::CertifyQuadraticFacetPolicyCoverage(
                pair.accepted[0], pair.prepared[0],
                pair.quadratic[0], pair.half_thickness[0],
                pair.accepted[1], pair.prepared[1],
                pair.quadratic[1], pair.half_thickness[1],
                2e-7, pair.accepted_owners.data(),
                pair.accepted_owners.size(),
                exclusions.data(), exclusions.size(),
                resolved.work - 1, 20);
        EXPECT_EQ(
            capped.status,
            sct::NonlinearSeparationStatus::WorkExhausted);
        EXPECT_EQ(capped.work, resolved.work - 1);
        const auto status =
            static_cast<unsigned>(resolved.status);
        ASSERT_LT(status, after.size());
        ++after[status];
        policy_work += resolved.work;
        const bool resolved_policy =
            resolved.status ==
                sct::NonlinearSeparationStatus::
                    CertifiedSeparated ||
            resolved.status ==
                sct::NonlinearSeparationStatus::
                    CertifiedAcceptedCoverage ||
            resolved.status ==
                sct::NonlinearSeparationStatus::
                    CertifiedExactExclusion;
        const bool typed_failure =
            resolved.status ==
                sct::NonlinearSeparationStatus::
                    PotentialContact ||
            resolved.status ==
                sct::NonlinearSeparationStatus::
                    OwnerAmbiguity ||
            resolved.status ==
                sct::NonlinearSeparationStatus::
                    PossibleGeometricCrossing;
        unexplained += !resolved_policy && !typed_failure;
        for (const unsigned depth : {17u, 20u, 32u, 52u}) {
            const auto deep =
                sct::CertifyQuadraticFacetPolicyCoverage(
                    pair.accepted[0], pair.prepared[0],
                    pair.quadratic[0], pair.half_thickness[0],
                    pair.accepted[1], pair.prepared[1],
                    pair.quadratic[1], pair.half_thickness[1],
                    2e-7, pair.accepted_owners.data(),
                    pair.accepted_owners.size(),
                    exclusions.data(), exclusions.size(),
                    4095, depth);
            std::cout << "LINEAR_FIXTURE_DEPTH"
                      << " depth=" << depth
                      << " status="
                      << static_cast<unsigned>(deep.status)
                      << " work=" << deep.work
                      << " deepest=" << deep.deepest
                      << " separated_cells="
                      << deep.separated_cells
                      << " covered_cells=" << deep.covered_cells
                      << " intersection=" << deep.has_intersection
                      << " unresolved_path="
                      << deep.unresolved_path
                      << " unresolved_depth="
                      << deep.unresolved_depth
                      << '\n';
            EXPECT_EQ(
                deep.status,
                sct::NonlinearSeparationStatus::PotentialContact);
            EXPECT_EQ(deep.work, depth + 9u);
            EXPECT_EQ(deep.deepest, depth);
            EXPECT_EQ(deep.covered_cells, 8u);
            EXPECT_EQ(deep.separated_cells, 0u);
            EXPECT_TRUE(deep.has_unresolved_cell);
            EXPECT_EQ(deep.unresolved_depth, depth);
            EXPECT_EQ(
                deep.unresolved_path,
                std::uint64_t{10067} << (depth - 17));
        }
        std::size_t seam_owners = 0;
        std::size_t exact_pair_owners = 0;
        for (const auto& evidence : pair.accepted_policy) {
            seam_owners += evidence.ledger_key_matches &&
                !evidence.ledger_exact_pair_matches;
            exact_pair_owners +=
                evidence.ledger_exact_pair_matches;
        }
        EXPECT_EQ(pair.accepted_owners.size(), 3u);
        EXPECT_EQ(seam_owners, 1u);
        EXPECT_EQ(exact_pair_owners, 0u);
        EXPECT_TRUE(exclusions.empty());
        std::cout << "LINEAR_FIXTURE_DIAGNOSIS"
                  << " first="
                  << pair.prepared[0].key.parent_eid << ":"
                  << pair.prepared[0].key.local_facet
                  << " second="
                  << pair.prepared[1].key.parent_eid << ":"
                  << pair.prepared[1].key.local_facet
                  << " common_translation="
                  << residual.exact_common_translation
                  << " residual_status="
                  << static_cast<unsigned>(residual.status)
                  << " persistent_status="
                  << static_cast<unsigned>(persistent.status)
                  << " owners=" << pair.accepted_owners.size()
                  << " seam_owners=" << seam_owners
                  << " exact_pair_owners=" << exact_pair_owners
                  << " exclusions=" << exclusions.size()
                  << " accepted_mask=" << pair.accepted_mask
                  << " prepared_mask=" << pair.prepared_mask
                  << " accepted_intersections="
                  << pair.accepted_intersections.size()
                  << " prepared_intersections="
                  << pair.prepared_intersections.size()
                  << " status="
                  << static_cast<unsigned>(resolved.status)
                  << " work=" << resolved.work
                  << " deepest=" << resolved.deepest
                  << " separated_cells="
                  << resolved.separated_cells
                  << " covered_cells=" << resolved.covered_cells
                  << " intersection=" << resolved.has_intersection
                  << '\n';
        nonlinear_fixture::HashUnsigned(status, &digest);
        nonlinear_fixture::HashUnsigned(
            resolved.work, &digest);
        nonlinear_fixture::HashUnsigned(
            resolved.proof_digest, &digest);
    }
    EXPECT_EQ(unexplained, 0u);
    EXPECT_EQ(
        after[static_cast<unsigned>(
            sct::NonlinearSeparationStatus::PotentialContact)],
        1u);
    EXPECT_EQ(
        std::accumulate(
            after.begin(), after.end(), std::size_t{0}),
        fixture::ExpectedPairs);
    EXPECT_EQ(digest, fixture::ExpectedPolicyResultDigest);
    EXPECT_EQ(fixture::SourceHash(data.pairs), source_before);
    const double runtime =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
    std::cout << "LINEAR_FIXTURE_RESULT"
              << " pairs=" << data.pairs.size()
              << " unexplained=" << unexplained
              << " baseline_work=" << baseline_work
              << " policy_work=" << policy_work
              << " digest=" << digest
              << " payload_bytes=" << data.payload_bytes
              << " runtime_s=" << runtime;
    for (std::size_t status = 0; status < after.size(); ++status)
        std::cout << " status" << status << "=" << after[status];
    std::cout << '\n';
}

}  // namespace
}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test
