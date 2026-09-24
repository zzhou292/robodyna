#include "LinearCoverageFixture.h"

#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/RepresentedIntervalCrossing.h"
#include "lib_src/collision/represented_interval_crossing/RelativeSeparationQualification.h"
#include "lib_utest/qualification/represented_interval_crossing/ResultAssertions.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
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
    std::size_t native_work = 0;
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
        const auto current = crossing.results();
        ASSERT_TRUE(current.complete);
        ASSERT_EQ(current.count, 1u);
        const auto comparison = contact::represented_interval_crossing::CompareRelativeSeparation(
            paths[0], paths[1], limits);
        ASSERT_EQ(comparison.status, contact::RepresentedIntervalStatus::Ok);
        ASSERT_TRUE(comparison.domain.eligible);
        EXPECT_FALSE(comparison.counters.saturated);
        EXPECT_GT(comparison.counters.first_face_separated, 0u);
        // Preserve the authenticated historical exhaustion and compare the
        // same native executor with/without the complete-cell face proof.
        EXPECT_EQ(pair.baseline_status, sct::NonlinearSeparationStatus::WorkExhausted);
        EXPECT_EQ(pair.baseline_work, 4095u);
        ASSERT_EQ(comparison.legacy.classification,
            contact::RepresentedIntervalClassification::Unresolved);
        ASSERT_EQ(comparison.legacy.reason,
            contact::RepresentedIntervalReason::WorkExhausted);
        ASSERT_EQ(comparison.legacy.work, pair.baseline_work);
        ASSERT_EQ(comparison.current.classification,
            contact::RepresentedIntervalClassification::CertifiedSeparated);
        ASSERT_EQ(comparison.current.reason, contact::RepresentedIntervalReason::None);
        ASSERT_EQ(comparison.current.work, 1u);
        represented_interval_test::SameResult(current.data[0], comparison.current);
        baseline_work += comparison.legacy.work;
        native_work += current.data[0].work;
        // Zero-thickness separation does not replace the finite-thickness
        // accepted-VF transition proof below; its complete assertions remain.

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
            sct::NonlinearSeparationStatus::
                CertifiedAcceptedCoverage);
        EXPECT_EQ(resolved.work, 1u);
        EXPECT_EQ(resolved.deepest, 0u);
        EXPECT_EQ(resolved.covered_cells, 1u);
        EXPECT_EQ(resolved.closed_covered_cells, 1u);
        EXPECT_EQ(resolved.separated_cells, 0u);
        EXPECT_FALSE(resolved.has_intersection);
        EXPECT_FALSE(resolved.has_unresolved_cell);
        EXPECT_TRUE(resolved.has_contact_transition);
        EXPECT_FALSE(resolved.transition_time_exact);
        EXPECT_TRUE(
            resolved.transition_zero_geometry_separated);
        EXPECT_EQ(resolved.accepted_certificate, 0u);
        EXPECT_EQ(resolved.accepted_source_order, 931u);
        EXPECT_EQ(
            resolved.transition_feature.kind,
            contact::RepresentedFeatureKind::VertexFace);
        EXPECT_EQ(resolved.transition_time_depth, 52u);
        EXPECT_EQ(
            resolved.transition_time_lower_numerator,
            345915413587096ull);
        EXPECT_GT(
            resolved.transition_time_lower_numerator,
            std::uint64_t{10067} << 35);
        EXPECT_LT(
            resolved.transition_time_lower_numerator,
            std::uint64_t{10068} << 35);
        EXPECT_EQ(repeated.status, resolved.status);
        EXPECT_EQ(repeated.work, resolved.work);
        EXPECT_EQ(
            repeated.proof_digest, resolved.proof_digest);
        EXPECT_EQ(reversed.status, resolved.status);
        EXPECT_EQ(reversed.work, resolved.work);
        EXPECT_EQ(reversed.deepest, resolved.deepest);
        EXPECT_EQ(
            reversed.accepted_source_order,
            resolved.accepted_source_order);
        EXPECT_EQ(
            reversed.transition_time_lower_numerator,
            resolved.transition_time_lower_numerator);
        EXPECT_EQ(
            reversed.transition_time_depth,
            resolved.transition_time_depth);
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
            permuted.accepted_source_order,
            resolved.accepted_source_order);
        EXPECT_EQ(
            permuted.transition_time_lower_numerator,
            resolved.transition_time_lower_numerator);
        EXPECT_EQ(
            permuted.unresolved_path, resolved.unresolved_path);
        const auto no_owner =
            sct::CertifyQuadraticFacetPolicyCoverage(
                pair.accepted[0], pair.prepared[0],
                pair.quadratic[0], pair.half_thickness[0],
                pair.accepted[1], pair.prepared[1],
                pair.quadratic[1], pair.half_thickness[1],
                2e-7, nullptr, 0,
                exclusions.data(), exclusions.size(),
                4095, 20);
        EXPECT_EQ(
            no_owner.status,
            sct::NonlinearSeparationStatus::MissingAcceptedOwner);
        auto one_bit_owner = pair.accepted_owners[0];
        one_bit_owner.discovery.key.vertex_face.vertex.
            source_instance_id ^= 1;
        one_bit_owner.event.feature =
            one_bit_owner.discovery.key;
        const auto one_bit =
            sct::CertifyQuadraticFacetPolicyCoverage(
                pair.accepted[0], pair.prepared[0],
                pair.quadratic[0], pair.half_thickness[0],
                pair.accepted[1], pair.prepared[1],
                pair.quadratic[1], pair.half_thickness[1],
                2e-7, &one_bit_owner, 1,
                exclusions.data(), exclusions.size(),
                4095, 20);
        EXPECT_EQ(
            one_bit.status,
            sct::NonlinearSeparationStatus::MissingAcceptedOwner);
        const double perturbed_thickness = std::nextafter(
            pair.half_thickness[0],
            std::numeric_limits<double>::infinity());
        const auto one_bit_geometry =
            sct::CertifyQuadraticFacetPolicyCoverage(
                pair.accepted[0], pair.prepared[0],
                pair.quadratic[0], perturbed_thickness,
                pair.accepted[1], pair.prepared[1],
                pair.quadratic[1], pair.half_thickness[1],
                2e-7, pair.accepted_owners.data(),
                pair.accepted_owners.size(),
                exclusions.data(), exclusions.size(),
                4095, 20);
        EXPECT_EQ(
            one_bit_geometry.status,
            sct::NonlinearSeparationStatus::
                CertifiedAcceptedCoverage);
        EXPECT_TRUE(one_bit_geometry.has_contact_transition);
        EXPECT_NE(
            one_bit_geometry.transition_time_lower_numerator,
            resolved.transition_time_lower_numerator);
        EXPECT_NE(
            one_bit_geometry.proof_digest,
            resolved.proof_digest);
        const auto zero_budget =
            sct::CertifyQuadraticFacetPolicyCoverage(
                pair.accepted[0], pair.prepared[0],
                pair.quadratic[0], pair.half_thickness[0],
                pair.accepted[1], pair.prepared[1],
                pair.quadratic[1], pair.half_thickness[1],
                2e-7, pair.accepted_owners.data(),
                pair.accepted_owners.size(),
                exclusions.data(), exclusions.size(),
                0, 20);
        EXPECT_EQ(
            zero_budget.status,
            sct::NonlinearSeparationStatus::InvalidInput);
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
                sct::NonlinearSeparationStatus::
                    CertifiedAcceptedCoverage);
            EXPECT_EQ(deep.work, 1u);
            EXPECT_EQ(deep.deepest, 0u);
            EXPECT_EQ(deep.covered_cells, 1u);
            EXPECT_EQ(deep.closed_covered_cells, 1u);
            EXPECT_EQ(deep.separated_cells, 0u);
            EXPECT_FALSE(deep.has_unresolved_cell);
            EXPECT_TRUE(deep.has_contact_transition);
            EXPECT_TRUE(
                deep.transition_zero_geometry_separated);
            EXPECT_EQ(deep.transition_time_depth, 52u);
            EXPECT_EQ(
                deep.transition_time_lower_numerator,
                resolved.transition_time_lower_numerator);
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
        for (std::size_t owner = 0;
             owner < pair.accepted_owners.size(); ++owner) {
            const auto& certificate = pair.accepted_owners[owner];
            const auto single =
                sct::CertifyQuadraticFacetPolicyCoverage(
                    pair.accepted[0], pair.prepared[0],
                    pair.quadratic[0], pair.half_thickness[0],
                    pair.accepted[1], pair.prepared[1],
                    pair.quadratic[1], pair.half_thickness[1],
                    2e-7, &certificate, 1,
                    nullptr, 0, 4095, 52);
            if (owner == 0) {
                EXPECT_EQ(
                    certificate.kind,
                    sct::AcceptedEventCertificateKind::VertexFace);
                EXPECT_EQ(certificate.event.source_order, 931u);
                EXPECT_EQ(
                    certificate.discovery.triangles[0].parent_eid,
                    2142378u);
                EXPECT_EQ(
                    certificate.discovery.triangles[0].local_facet,
                    0u);
                EXPECT_EQ(
                    certificate.discovery.triangles[1].parent_eid,
                    2230072u);
                EXPECT_EQ(
                    certificate.discovery.triangles[1].local_facet,
                    1u);
                EXPECT_EQ(
                    certificate.discovery.local_features[0], 1u);
                EXPECT_EQ(
                    certificate.discovery.local_features[1], 3u);
                ASSERT_EQ(
                    certificate.discovery.key.kind,
                    contact::FixedTriangleCandidateKind::VertexFace);
                EXPECT_EQ(
                    certificate.discovery.key.vertex_face.target.kind,
                    contact::FixedTriangleStratumKind::Face);
                EXPECT_EQ(
                    contact::fixed_triangle_features::Compare(
                        certificate.discovery.key.vertex_face.
                            target.face,
                        pair.prepared[1].key),
                    0);
                const auto prepared_feature = std::find_if(
                    pair.prepared_features.begin(),
                    pair.prepared_features.end(),
                    [&](const auto& feature) {
                        return contact::fixed_triangle_features::Compare(
                                   feature.key,
                                   certificate.discovery.key) == 0;
                    });
                ASSERT_NE(
                    prepared_feature, pair.prepared_features.end());
                for (const double weight :
                     prepared_feature->face_weights)
                    EXPECT_GT(weight, 0);
                const auto& vertex_key =
                    certificate.discovery.key.vertex_face.vertex;
                unsigned source_vertex = 3;
                for (unsigned vertex = 0; vertex < 3; ++vertex)
                    if (contact::fixed_triangle_features::Compare(
                            pair.prepared[0].vertex_keys[vertex],
                            vertex_key) == 0)
                        source_vertex = vertex;
                ASSERT_LT(source_vertex, 3u);
                EXPECT_EQ(
                    single.status,
                    sct::NonlinearSeparationStatus::
                        CertifiedAcceptedCoverage);
                EXPECT_TRUE(single.has_contact_transition);
                EXPECT_EQ(
                    single.transition_time_lower_numerator,
                    345915413587096ull);
            } else {
                EXPECT_EQ(
                    single.status,
                    sct::NonlinearSeparationStatus::
                        MissingAcceptedOwner);
            }
        }
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
            sct::NonlinearSeparationStatus::
                CertifiedAcceptedCoverage)],
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
              << " native_work=" << native_work
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
