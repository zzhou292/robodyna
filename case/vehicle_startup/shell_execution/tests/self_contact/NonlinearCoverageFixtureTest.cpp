#include "NonlinearCoverageFixture.h"

#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
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

enum class AcceptedPairCategory : unsigned {
    InactiveParent,
    SameRigidGroupOrSupport,
    LocalIncidenceOrRegularity,
    TiedOrCinExclusion,
    UnsupportedForceArea,
    DistanceOrRepresentationCertificate,
    SeamOrDedupOwnerOutsidePair,
    FeatureWeightNormalization,
    ActualLedgerOmissionOrLookupBug,
    Count,
};

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

double FromBits(std::uint64_t bits) {
    double result = 0;
    std::memcpy(&result, &bits, sizeof(result));
    return result;
}

bool PhysicallyInside(
    const contact::FixedTriangleFeatureCandidate& feature,
    double thickness) {
    return (feature.distance_m - thickness) <=
        feature.representation_error_m;
}

AcceptedPairCategory ClassifyAcceptedPair(
    const fixture::Pair& pair,
    std::array<std::size_t, static_cast<unsigned>(
        AcceptedPairCategory::Count)>* feature_categories,
    bool* nonexcluded_ownerless) {
    const double thickness =
        pair.half_thickness[0] + pair.half_thickness[1];
    bool inactive = false;
    bool rigid = false;
    bool local = false;
    bool tied = false;
    bool area = false;
    bool distance = false;
    bool seam = false;
    bool weight = false;
    bool omission = false;
    *nonexcluded_ownerless = false;
    for (std::size_t index = 0;
         index < pair.accepted_features.size(); ++index) {
        const auto& feature = pair.accepted_features[index];
        const auto& evidence = pair.accepted_policy[index];
        if (!PhysicallyInside(feature, thickness)) {
            distance = true;
            ++(*feature_categories)[static_cast<unsigned>(
                AcceptedPairCategory::
                    DistanceOrRepresentationCertificate)];
            continue;
        }
        AcceptedPairCategory category =
            AcceptedPairCategory::ActualLedgerOmissionOrLookupBug;
        if (!evidence.active[0] || !evidence.active[1] ||
            evidence.pair_status ==
                contact::SelfContactPairStatus::InactiveParent) {
            inactive = true;
            category = AcceptedPairCategory::InactiveParent;
        } else if (evidence.pair_status ==
                       contact::SelfContactPairStatus::
                           ExcludedSameRigidGroup ||
                   (evidence.endpoint_support[0].status ==
                        contact::SelfContactSupportStatus::
                            CompleteRigidGroup &&
                    evidence.endpoint_support[1].status ==
                        contact::SelfContactSupportStatus::
                            CompleteRigidGroup &&
                    evidence.endpoint_support[0].
                            complete_rigid_group ==
                        evidence.endpoint_support[1].
                            complete_rigid_group)) {
            rigid = true;
            category =
                AcceptedPairCategory::SameRigidGroupOrSupport;
        } else if (evidence.local_incidence ||
                   evidence.pair_status ==
                       contact::SelfContactPairStatus::
                           ExcludedLocalIncidence ||
                   evidence.pair_status ==
                       contact::SelfContactPairStatus::
                           ExcludedRegularOwnParent) {
            local = true;
            category =
                AcceptedPairCategory::LocalIncidenceOrRegularity;
        } else if (evidence.tied !=
                       contact::SelfContactTiedStatus::NotRelated ||
                   evidence.pair_status ==
                       contact::SelfContactPairStatus::
                           UnsupportedCinSecondary ||
                   evidence.pair_status ==
                       contact::SelfContactPairStatus::
                           UnresolvedTiedSupportNotExcluded) {
            tied = true;
            category = AcceptedPairCategory::TiedOrCinExclusion;
        } else if (evidence.pair_status ==
                       contact::SelfContactPairStatus::
                           UnadmittedEdgeEdgeForceArea ||
                   evidence.disposition ==
                       sct::AcceptedFeatureDisposition::
                           UnsupportedForceArea) {
            area = true;
            category = AcceptedPairCategory::UnsupportedForceArea;
        } else if (evidence.disposition ==
                   sct::AcceptedFeatureDisposition::
                       FeatureWeightNormalization) {
            weight = true;
            category =
                AcceptedPairCategory::FeatureWeightNormalization;
        } else if (evidence.admitted) {
            if (evidence.ledger_key_matches &&
                !evidence.ledger_exact_pair_matches) {
                seam = true;
                category = AcceptedPairCategory::
                    SeamOrDedupOwnerOutsidePair;
            } else {
                omission = true;
                *nonexcluded_ownerless =
                    !evidence.ledger_key_matches;
                category =
                    AcceptedPairCategory::
                        ActualLedgerOmissionOrLookupBug;
            }
        } else if (evidence.distance_separated) {
            distance = true;
            category = AcceptedPairCategory::
                DistanceOrRepresentationCertificate;
        } else {
            omission = true;
            *nonexcluded_ownerless = true;
        }
        ++(*feature_categories)[static_cast<unsigned>(category)];
    }
    if (omission)
        return AcceptedPairCategory::
            ActualLedgerOmissionOrLookupBug;
    if (seam)
        return AcceptedPairCategory::SeamOrDedupOwnerOutsidePair;
    if (inactive) return AcceptedPairCategory::InactiveParent;
    if (rigid)
        return AcceptedPairCategory::SameRigidGroupOrSupport;
    if (local)
        return AcceptedPairCategory::LocalIncidenceOrRegularity;
    if (tied) return AcceptedPairCategory::TiedOrCinExclusion;
    if (area) return AcceptedPairCategory::UnsupportedForceArea;
    if (weight)
        return AcceptedPairCategory::FeatureWeightNormalization;
    return AcceptedPairCategory::
        DistanceOrRepresentationCertificate;
}

std::vector<sct::AcceptedEventCertificate>
AcceptedOwnersFromPolicy(const fixture::Pair& pair) {
    std::vector<sct::AcceptedEventCertificate> result =
        pair.accepted_owners;
    for (std::size_t index = 0;
         index < pair.accepted_features.size(); ++index) {
        const auto& evidence = pair.accepted_policy[index];
        if (!evidence.ledger_key_matches) continue;
        const auto already = std::find_if(
            result.begin(), result.end(),
            [&](const auto& value) {
                return value.event.source_order ==
                    evidence.first_ledger_source_order;
            });
        if (already != result.end()) continue;
        sct::AcceptedEventCertificate owner;
        const auto& feature = pair.accepted_features[index];
        owner.kind = feature.key.kind ==
                contact::FixedTriangleCandidateKind::VertexFace
            ? sct::AcceptedEventCertificateKind::VertexFace
            : sct::AcceptedEventCertificateKind::EdgeEdge;
        owner.discovery = feature;
        owner.discovery.triangles[0] =
            evidence.first_ledger_triangles[0];
        owner.discovery.triangles[1] =
            evidence.first_ledger_triangles[1];
        owner.event.feature = feature.key;
        owner.event.source_order =
            evidence.first_ledger_source_order;
        owner.event.classification.kind = evidence.kind;
        owner.event.classification.status =
            owner.kind ==
                    sct::AcceptedEventCertificateKind::VertexFace
                ? contact::SelfContactPairStatus::
                      AdmittedVertexFace
                : contact::SelfContactPairStatus::
                      AdmittedEdgeEdge;
        owner.event.classification.active[0] = true;
        owner.event.classification.active[1] = true;
        owner.event.classification.parent[0] =
            evidence.first_ledger_parent[0];
        owner.event.classification.parent[1] =
            evidence.first_ledger_parent[1];
        result.push_back(owner);
    }
    return result;
}

std::vector<sct::AcceptedFeatureExclusionCertificate>
AcceptedExclusionsFromPolicy(const fixture::Pair& pair) {
    std::vector<sct::AcceptedFeatureExclusionCertificate> result;
    for (std::size_t index = 0;
         index < pair.accepted_features.size(); ++index) {
        const auto& evidence = pair.accepted_policy[index];
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
            pair.accepted_features[index],
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
     FrozenRosterResolvesWithAuthenticatedPolicyEvidence) {
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
    ASSERT_EQ(
        data.phase.accepted_label,
        fixture::PhaseLabel::AcceptedOwner);
    ASSERT_EQ(
        data.phase.prepared_label,
        fixture::PhaseLabel::PreparedCandidate);
    ASSERT_EQ(
        data.phase.accepted_epoch,
        data.phase.prepared_base_epoch);
    ASSERT_TRUE(SameBits(
        FromBits(data.phase.accepted_time_bits),
        FromBits(data.phase.prepared_base_time_bits)));
    ASSERT_EQ(
        FromBits(data.phase.prepared_time_bits) -
            FromBits(data.phase.prepared_base_time_bits),
        2e-7);
    ASSERT_EQ(data.phase.accepted_temporal_scheme, 1u);
    ASSERT_EQ(data.phase.prepared_temporal_scheme, 1u);
    ASSERT_EQ(data.phase.accepted_velocity_phase, 0u);
    ASSERT_EQ(data.phase.prepared_velocity_phase, 1u);

    const auto source_before =
        fixture::SourceHash(data.pairs);
    std::array<std::size_t, 10> statuses{};
    std::array<std::size_t, static_cast<unsigned>(
        AcceptedPairCategory::Count)> pair_categories{};
    std::array<std::size_t, static_cast<unsigned>(
        AcceptedPairCategory::Count)> feature_categories{};
    std::size_t nonexcluded_ownerless = 0;
    std::size_t work = 0;
    unsigned deepest = 0;
    std::uint64_t result_digest = 1469598103934665603ull;
    std::size_t unresolved = 0;
    for (std::size_t pair_index = 0;
         pair_index < data.pairs.size(); ++pair_index) {
        const auto& pair = data.pairs[pair_index];
        ASSERT_EQ(
            pair.accepted_policy.size(),
            pair.accepted_features.size());
        double accepted_minimum =
            std::numeric_limits<double>::infinity();
        double prepared_minimum =
            std::numeric_limits<double>::infinity();
        for (const auto& feature : pair.accepted_features)
            accepted_minimum = std::min(
                accepted_minimum, feature.distance_m);
        for (const auto& feature : pair.prepared_features)
            prepared_minimum = std::min(
                prepared_minimum, feature.distance_m);
        const double thickness =
            pair.half_thickness[0] +
            pair.half_thickness[1];
        EXPECT_LT(accepted_minimum, thickness)
            << "pair=" << pair_index;
        EXPECT_LT(prepared_minimum, thickness)
            << "pair=" << pair_index;
        if (!pair_index) {
            EXPECT_EQ(
                accepted_minimum,
                0.00086444683106572876);
            EXPECT_EQ(thickness, 0.001065);
        }
        bool pair_nonexcluded_ownerless = false;
        const auto accepted_category =
            ClassifyAcceptedPair(
                pair, &feature_categories,
                &pair_nonexcluded_ownerless);
        ++pair_categories[
            static_cast<unsigned>(accepted_category)];
        nonexcluded_ownerless +=
            pair_nonexcluded_ownerless;
        if (accepted_category !=
            AcceptedPairCategory::SameRigidGroupOrSupport) {
            std::cout << "ACCEPTED_POLICY_PAIR"
                      << " pair=" << pair_index
                      << " first="
                      << pair.accepted[0].key.parent_eid
                      << ":" << pair.accepted[0].key.local_facet
                      << " second="
                      << pair.accepted[1].key.parent_eid
                      << ":" << pair.accepted[1].key.local_facet
                      << " category="
                      << static_cast<unsigned>(accepted_category)
                      << " accepted_minimum="
                      << std::setprecision(17)
                      << accepted_minimum
                      << " prepared_minimum="
                      << prepared_minimum
                      << " thickness=" << thickness << '\n';
            for (std::size_t feature_index = 0;
                 feature_index < pair.accepted_features.size();
                 ++feature_index) {
                const auto& feature =
                    pair.accepted_features[feature_index];
                if (!PhysicallyInside(feature, thickness))
                    continue;
                const auto& evidence =
                    pair.accepted_policy[feature_index];
                std::cout
                    << "ACCEPTED_POLICY_FEATURE"
                    << " pair=" << pair_index
                    << " feature=" << feature_index
                    << " kind="
                    << static_cast<unsigned>(feature.key.kind)
                    << " target_kind="
                    << (feature.key.kind ==
                                contact::FixedTriangleCandidateKind::
                                    VertexFace
                            ? static_cast<unsigned>(
                                  feature.key.vertex_face.target.kind)
                            : 99u)
                    << " local=" << feature.local_features[0]
                    << "," << feature.local_features[1]
                    << " distance=" << feature.distance_m
                    << " error="
                    << feature.representation_error_m
                    << " disposition="
                    << static_cast<unsigned>(
                           evidence.disposition)
                    << " report="
                    << static_cast<unsigned>(
                           evidence.report_status)
                    << " pair_status="
                    << static_cast<unsigned>(
                           evidence.pair_status)
                    << " support="
                    << static_cast<unsigned>(
                           evidence.endpoint_support[0].status)
                    << ":"
                    << evidence.endpoint_support[0].
                           complete_rigid_group
                    << ","
                    << static_cast<unsigned>(
                           evidence.endpoint_support[1].status)
                    << ":"
                    << evidence.endpoint_support[1].
                           complete_rigid_group
                    << " parent=" << evidence.parent[0]
                    << "," << evidence.parent[1]
                    << " active=" << evidence.active[0]
                    << "," << evidence.active[1]
                    << " local_incidence="
                    << evidence.local_incidence
                    << " tied="
                    << static_cast<unsigned>(evidence.tied)
                    << " ledger_key="
                    << evidence.ledger_key_matches
                    << " ledger_exact="
                    << evidence.ledger_exact_pair_matches
                    << " ledger_lower="
                    << evidence.ledger_canonical_lower_matches
                    << " ledger_foreign="
                    << evidence.ledger_foreign_owner_matches;
                if (evidence.ledger_key_matches)
                    std::cout
                        << " first_owner="
                        << evidence.first_ledger_triangles[0].
                               parent_eid
                        << ":"
                        << evidence.first_ledger_triangles[0].
                               local_facet
                        << ","
                        << evidence.first_ledger_triangles[1].
                               parent_eid
                        << ":"
                        << evidence.first_ledger_triangles[1].
                               local_facet
                        << " source_order="
                        << evidence.first_ledger_source_order;
                if (feature.key.kind ==
                        contact::FixedTriangleCandidateKind::
                            VertexFace &&
                    feature.key.vertex_face.target.kind ==
                        contact::FixedTriangleStratumKind::Edge) {
                    const auto has_vertex = [](
                        const auto& triangle, const auto& key) {
                        for (const auto& value :
                             triangle.vertex_keys)
                            if (contact::fixed_triangle_features::
                                    Compare(value, key) == 0)
                                return true;
                        return false;
                    };
                    const auto has_edge = [](
                        const auto& triangle, const auto& key) {
                        for (const auto& value :
                             triangle.edge_keys)
                            if (contact::fixed_triangle_features::
                                    Compare(value, key) == 0)
                                return true;
                        return false;
                    };
                    std::cout
                        << " source_vertex_sides="
                        << has_vertex(
                               pair.prepared[0],
                               feature.key.vertex_face.vertex)
                        << ","
                        << has_vertex(
                               pair.prepared[1],
                               feature.key.vertex_face.vertex)
                        << " target_edge_sides="
                        << has_edge(
                               pair.prepared[0],
                               feature.key.vertex_face.target.edge)
                        << ","
                        << has_edge(
                               pair.prepared[1],
                               feature.key.vertex_face.target.edge)
                        << " target_t="
                        << feature.edge_parameters[0];
                }
                std::cout << '\n';
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
        const auto replay_owners =
            AcceptedOwnersFromPolicy(pair);
        const auto replay_exclusions =
            AcceptedExclusionsFromPolicy(pair);
        const auto certify = [&](const auto* owners,
                                 std::size_t owner_count,
                                 bool reverse) {
            return reverse
                ? sct::CertifyQuadraticFacetPolicyCoverage(
                      pair.accepted[1], pair.prepared[1],
                      pair.quadratic[1],
                      pair.half_thickness[1],
                      pair.accepted[0], pair.prepared[0],
                      pair.quadratic[0],
                      pair.half_thickness[0], 2e-7,
                      owners, owner_count,
                      replay_exclusions.data(),
                      replay_exclusions.size(), 4095, 20)
                : sct::CertifyQuadraticFacetPolicyCoverage(
                      pair.accepted[0], pair.prepared[0],
                      pair.quadratic[0],
                      pair.half_thickness[0],
                      pair.accepted[1], pair.prepared[1],
                      pair.quadratic[1],
                      pair.half_thickness[1], 2e-7,
                      owners, owner_count,
                      replay_exclusions.data(),
                      replay_exclusions.size(), 4095, 20);
        };
        const auto result = certify(
            replay_owners.data(),
            replay_owners.size(), false);
        const auto reversed = certify(
            replay_owners.data(),
            replay_owners.size(), true);
        EXPECT_EQ(reversed.status, result.status)
            << "pair=" << pair_index;
        EXPECT_EQ(reversed.work, result.work)
            << "pair=" << pair_index;
        EXPECT_EQ(reversed.deepest, result.deepest)
            << "pair=" << pair_index;
        EXPECT_EQ(reversed.proof_digest, result.proof_digest)
            << "pair=" << pair_index;
        auto permuted_owners = replay_owners;
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
                replay_owners.data(),
                replay_owners.size(), false);
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
                    CertifiedAcceptedCoverage ||
            result.status ==
                sct::NonlinearSeparationStatus::
                    CertifiedExactExclusion;
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
                << replay_owners.size()
                << " exclusions="
                << replay_exclusions.size()
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
    EXPECT_EQ(unresolved, 0u);
    EXPECT_EQ(
        statuses[static_cast<unsigned>(
            sct::NonlinearSeparationStatus::
                MissingAcceptedOwner)],
        0u);
    EXPECT_EQ(
        statuses[static_cast<unsigned>(
            sct::NonlinearSeparationStatus::
                CertifiedAcceptedCoverage)],
        5u);
    EXPECT_EQ(
        statuses[static_cast<unsigned>(
            sct::NonlinearSeparationStatus::
                CertifiedExactExclusion)],
        312u);
    EXPECT_EQ(
        std::accumulate(
            statuses.begin(), statuses.end(),
            std::size_t{0}),
        fixture::ExpectedPairs);
    EXPECT_EQ(nonexcluded_ownerless, 0u);
    EXPECT_EQ(
        pair_categories[static_cast<unsigned>(
            AcceptedPairCategory::SameRigidGroupOrSupport)],
        312u);
    EXPECT_EQ(
        pair_categories[static_cast<unsigned>(
            AcceptedPairCategory::SeamOrDedupOwnerOutsidePair)],
        1u);
    EXPECT_EQ(
        pair_categories[static_cast<unsigned>(
            AcceptedPairCategory::
                ActualLedgerOmissionOrLookupBug)],
        4u);
    EXPECT_EQ(
        result_digest,
        fixture::ExpectedPolicyResultDigest);
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
              << " ledger_covered="
              << statuses[static_cast<unsigned>(
                     sct::NonlinearSeparationStatus::
                         CertifiedAcceptedCoverage)]
              << " exact_excluded="
              << statuses[static_cast<unsigned>(
                     sct::NonlinearSeparationStatus::
                         CertifiedExactExclusion)]
              << " true_nonexcluded_ownerless="
              << nonexcluded_ownerless
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
    for (std::size_t category = 0;
         category < pair_categories.size(); ++category)
        std::cout << " pair_category" << category
                  << "=" << pair_categories[category]
                  << " feature_category" << category
                  << "=" << feature_categories[category];
    std::cout << " accepted_phase="
              << static_cast<std::uint64_t>(
                     data.phase.accepted_label)
              << " prepared_phase="
              << static_cast<std::uint64_t>(
                     data.phase.prepared_label)
              << " accepted_epoch="
              << data.phase.accepted_epoch
              << " prepared_base_epoch="
              << data.phase.prepared_base_epoch
              << " accepted_time="
              << FromBits(data.phase.accepted_time_bits)
              << " prepared_base_time="
              << FromBits(data.phase.prepared_base_time_bits)
              << " prepared_time="
              << FromBits(data.phase.prepared_time_bits)
              << " accepted_temporal_scheme="
              << data.phase.accepted_temporal_scheme
              << " prepared_temporal_scheme="
              << data.phase.prepared_temporal_scheme
              << " accepted_velocity_phase="
              << data.phase.accepted_velocity_phase
              << " prepared_velocity_phase="
              << data.phase.prepared_velocity_phase;
    std::cout << '\n';
}

}  // namespace
}  // namespace crash::cases::vehicle_startup::shell_execution::
   // self_contact_test
