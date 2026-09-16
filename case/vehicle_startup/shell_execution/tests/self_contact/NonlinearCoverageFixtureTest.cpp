#include "NonlinearCoverageFixture.h"

#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <numeric>
#include <vector>

#ifndef ROBO_NONLINEAR_FIXTURE_PATH
#error "ROBO_NONLINEAR_FIXTURE_PATH is required"
#endif

namespace crash::cases::vehicle_startup::shell_execution::
    self_contact_test {
namespace {

namespace contact = tlfea::contact;
namespace fixture = nonlinear_fixture;
namespace sct = tlfea::contact::self_contact_transaction;

bool SameBits(double first, double second) {
    std::uint64_t a = 0;
    std::uint64_t b = 0;
    std::memcpy(&a, &first, sizeof(a));
    std::memcpy(&b, &second, sizeof(b));
    return a == b;
}

bool SamePoint(contact::Vec3 first, contact::Vec3 second) {
    return SameBits(first.x, second.x) &&
        SameBits(first.y, second.y) &&
        SameBits(first.z, second.z);
}

bool SameFeature(
    const contact::FixedTriangleFeatureCandidate& first,
    const contact::FixedTriangleFeatureCandidate& second) {
    if (contact::fixed_triangle_features::Compare(
            first.key, second.key) != 0)
        return false;
    for (unsigned side = 0; side < 2; ++side)
        if (contact::fixed_triangle_features::Compare(
                first.triangles[side],
                second.triangles[side]) != 0 ||
            first.local_features[side] !=
                second.local_features[side] ||
            !SamePoint(first.points[side], second.points[side]) ||
            !SameBits(
                first.edge_parameters[side],
                second.edge_parameters[side]))
            return false;
    for (unsigned weight = 0; weight < 3; ++weight)
        if (!SameBits(
                first.face_weights[weight],
                second.face_weights[weight]))
            return false;
    return SameBits(first.distance_m, second.distance_m) &&
        SameBits(
            first.representation_error_m,
            second.representation_error_m);
}

void VerifyEndpoint(
    const contact::CurrentFixedTriangle triangles[2],
    std::uint16_t expected_mask,
    const std::vector<
        contact::FixedTriangleFeatureCandidate>& expected_features,
    const std::vector<
        contact::FixedTriangleIntersection>& expected_intersections) {
    contact::FixedTriangleFeatureTaskMask mask;
    ASSERT_EQ(
        contact::BuildFixedTriangleFeatureTaskMask(
            triangles[0], triangles[1], &mask),
        contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(mask.local_tasks, expected_mask);
    std::array<
        contact::FixedTriangleFeatureCandidate, 15> features;
    contact::fixed_triangle_features::PairFeatureResult result;
    ASSERT_EQ(
        contact::fixed_triangle_features::
            EvaluatePairFeaturesMaskedOnce(
                triangles[0], triangles[1], mask,
                features.data(), features.size(), &result),
        contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(result.feature_count, expected_features.size());
    for (std::size_t feature = 0;
         feature < expected_features.size(); ++feature)
        EXPECT_TRUE(SameFeature(
            features[feature], expected_features[feature]))
            << "feature=" << feature;
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
        expected_intersections.size());
    if (intersects) {
        EXPECT_TRUE(
            contact::fixed_triangle_features::
                SameIntersectionPair(
                    intersection,
                    expected_intersections[0]));
        EXPECT_EQ(
            intersection.kind,
            expected_intersections[0].kind);
        EXPECT_EQ(
            intersection.local_exclusion,
            expected_intersections[0].local_exclusion);
    }
}

TEST(VehicleSelfContactNonlinearFixture,
     FrozenRosterReturnsDeterministicIrreducibleReasons) {
    const auto started = std::chrono::steady_clock::now();
    const auto data = fixture::Read(
        ROBO_NONLINEAR_FIXTURE_PATH);
    ASSERT_EQ(data.schema_hash, fixture::SchemaHash());
    ASSERT_EQ(data.pairs.size(), fixture::ExpectedPairs);
    ASSERT_EQ(
        fixture::RosterDigest(data.pairs),
        fixture::ExpectedRosterDigest);
    ASSERT_EQ(
        data.nonlinear_roster_digest,
        fixture::ExpectedNonlinearRosterDigest);
    ASSERT_EQ(data.source_hash, fixture::ExpectedSourceHash);
    ASSERT_EQ(data.profile_hash, fixture::ExpectedProfileHash);
    ASSERT_EQ(data.dt_hash, fixture::ExpectedDtHash);
    ASSERT_EQ(data.payload_hash, fixture::ExpectedPayloadHash);
    ASSERT_EQ(data.payload_bytes, fixture::ExpectedPayloadBytes);

    const auto source_before =
        fixture::SourceHash(data.pairs);
    std::array<std::size_t, 9> statuses{};
    std::size_t work = 0;
    unsigned deepest = 0;
    std::uint64_t result_digest = 1469598103934665603ull;
    std::size_t unresolved = 0;
    for (std::size_t pair_index = 0;
         pair_index < data.pairs.size(); ++pair_index) {
        const auto& pair = data.pairs[pair_index];
        VerifyEndpoint(
            pair.accepted, pair.accepted_mask,
            pair.accepted_features,
            pair.accepted_intersections);
        VerifyEndpoint(
            pair.prepared, pair.prepared_mask,
            pair.prepared_features,
            pair.prepared_intersections);
        const auto certify = [&](const auto* owners,
                                 std::size_t owner_count,
                                 bool reverse) {
            return reverse
                ? sct::CertifyQuadraticFacetCoverage(
                      pair.accepted[1], pair.prepared[1],
                      pair.quadratic[1],
                      pair.half_thickness[1],
                      pair.accepted[0], pair.prepared[0],
                      pair.quadratic[0],
                      pair.half_thickness[0], 2e-7,
                      owners, owner_count, 4095, 20)
                : sct::CertifyQuadraticFacetCoverage(
                      pair.accepted[0], pair.prepared[0],
                      pair.quadratic[0],
                      pair.half_thickness[0],
                      pair.accepted[1], pair.prepared[1],
                      pair.quadratic[1],
                      pair.half_thickness[1], 2e-7,
                      owners, owner_count, 4095, 20);
        };
        const auto result = certify(
            pair.accepted_owners.data(),
            pair.accepted_owners.size(), false);
        const auto reversed = certify(
            pair.accepted_owners.data(),
            pair.accepted_owners.size(), true);
        EXPECT_EQ(reversed.status, result.status)
            << "pair=" << pair_index;
        EXPECT_EQ(reversed.work, result.work)
            << "pair=" << pair_index;
        EXPECT_EQ(reversed.deepest, result.deepest)
            << "pair=" << pair_index;
        EXPECT_EQ(reversed.proof_digest, result.proof_digest)
            << "pair=" << pair_index;
        auto permuted_owners = pair.accepted_owners;
        std::reverse(
            permuted_owners.begin(), permuted_owners.end());
        const auto permuted = certify(
            permuted_owners.data(),
            permuted_owners.size(), false);
        EXPECT_EQ(permuted.status, result.status)
            << "pair=" << pair_index;
        EXPECT_EQ(
            permuted.accepted_source_order,
            result.accepted_source_order)
            << "pair=" << pair_index;
        EXPECT_EQ(
            permuted.proof_digest, result.proof_digest)
            << "pair=" << pair_index;
        for (unsigned repeat = 0; repeat < 4; ++repeat) {
            const auto repeated = certify(
                pair.accepted_owners.data(),
                pair.accepted_owners.size(), false);
            EXPECT_EQ(repeated.status, result.status);
            EXPECT_EQ(repeated.work, result.work);
            EXPECT_EQ(
                repeated.proof_digest, result.proof_digest);
        }
        const auto status =
            static_cast<unsigned>(result.status);
        ASSERT_LT(status, statuses.size());
        ++statuses[status];
        work += result.work;
        deepest = std::max(deepest, result.deepest);
        fixture::HashPath(pair.prepared[0].key, &result_digest);
        fixture::HashPath(pair.prepared[1].key, &result_digest);
        fixture::HashUnsigned(status, &result_digest);
        fixture::HashUnsigned(result.work, &result_digest);
        fixture::HashUnsigned(result.deepest, &result_digest);
        fixture::HashUnsigned(
            result.accepted_source_order, &result_digest);
        fixture::HashUnsigned(
            result.proof_digest, &result_digest);
        const bool resolved =
            result.status ==
                sct::NonlinearSeparationStatus::
                    CertifiedSeparated ||
            result.status ==
                sct::NonlinearSeparationStatus::
                    CertifiedAcceptedCoverage;
        unresolved += !resolved;
        if (!resolved)
            std::cout
                << "NONLINEAR_FIXTURE_UNRESOLVED"
                << " pair=" << pair_index
                << " first="
                << pair.prepared[0].key.parent_eid
                << ":" << pair.prepared[0].key.local_facet
                << " second="
                << pair.prepared[1].key.parent_eid
                << ":" << pair.prepared[1].key.local_facet
                << " status=" << status
                << " work=" << result.work
                << " depth=" << result.deepest
                << " owners="
                << pair.accepted_owners.size()
                << " separated_cells="
                << result.separated_cells
                << " covered_cells="
                << result.covered_cells
                << " work_exhausted="
                << result.work_exhausted
                << " depth_exhausted="
                << result.depth_exhausted
                << '\n';
    }
    EXPECT_EQ(fixture::SourceHash(data.pairs), source_before);
    EXPECT_EQ(unresolved, fixture::ExpectedPairs);
    EXPECT_EQ(
        statuses[static_cast<unsigned>(
            sct::NonlinearSeparationStatus::
                MissingAcceptedOwner)],
        fixture::ExpectedPairs);
    EXPECT_EQ(
        std::accumulate(
            statuses.begin(), statuses.end(),
            std::size_t{0}),
        fixture::ExpectedPairs);
    const double elapsed =
        std::chrono::duration<double>(
            std::chrono::steady_clock::now() - started).count();
    std::cout << "NONLINEAR_FIXTURE_RESULT"
              << " pairs=" << data.pairs.size()
              << " unresolved=" << unresolved
              << " irreducible_missing_owner="
              << statuses[static_cast<unsigned>(
                     sct::NonlinearSeparationStatus::
                         MissingAcceptedOwner)]
              << " work=" << work
              << " depth=" << deepest
              << " result_digest=" << result_digest
              << " payload_bytes=" << data.payload_bytes
              << " payload_hash=" << data.payload_hash
              << " source_hash=" << data.source_hash
              << " schema_hash=" << data.schema_hash
              << " profile_hash=" << data.profile_hash
              << " dt_hash=" << data.dt_hash
              << " runtime_s=" << elapsed;
    for (std::size_t status = 0;
         status < statuses.size(); ++status)
        std::cout << " status" << status
                  << "=" << statuses[status];
    std::cout << '\n';
}

}  // namespace
}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test
