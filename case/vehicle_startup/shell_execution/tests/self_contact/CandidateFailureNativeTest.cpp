#include "CandidateFailureFixture.h"

#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/RepresentedIntervalCrossing.h"
#include "lib_src/collision/self_contact_transaction/TranslatedLocal.h"
#include "output/BoundedArrayJson.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdlib>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace {

contact::RepresentedTrianglePath AffinePath(
    const contact::CurrentFixedTriangle& accepted,
    const contact::CurrentFixedTriangle& prepared) {
    contact::RepresentedTrianglePath path;
    path.key = {prepared.key.source_instance_id, prepared.key.parent_eid,
                prepared.key.level, prepared.key.local_facet};
    path.motion = contact::RepresentedMotion::LinearNodalV1;
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
        path.vertices[vertex].key = prepared.vertex_keys[vertex];
        path.vertices[vertex].endpoint[0] = accepted.vertices[vertex];
        path.vertices[vertex].endpoint[1] = prepared.vertices[vertex];
        path.edge_keys[vertex] = prepared.edge_keys[vertex];
    }
    return path;
}

void CheckFrozenLocalIntersection(
    const contact::CurrentFixedTriangle triangles[2], std::uint16_t expected_mask,
    const std::vector<contact::FixedTriangleIntersection>& frozen) {
    ASSERT_EQ(frozen.size(), 1u);
    contact::FixedTriangleFeatureTaskMask mask;
    ASSERT_EQ(contact::BuildFixedTriangleFeatureTaskMask(triangles[0], triangles[1], &mask),
              contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_EQ(mask.local_tasks, expected_mask);
    contact::FixedTriangleIntersection actual;
    bool intersects = false;
    ASSERT_EQ(contact::fixed_triangle_features::ClassifyPairIntersection(
                  triangles[0], triangles[1], &actual, &intersects),
              contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_TRUE(intersects);
    ASSERT_TRUE(sct::Same(actual, frozen.front()));
    ASSERT_FALSE(contact::RequiresIntersectionAdmission(actual));
}

}  // namespace

TEST(CandidateFailureNativeReplay, PinnedAffineLocalPairRetainsNativeWholeIntervalProof) {
    const auto* manifest_name = std::getenv("ROBO_SELF_CONTACT_FAILURE_MANIFEST");
    const auto* expected_sha = std::getenv("ROBO_SELF_CONTACT_FAILURE_SHA256");
    ASSERT_TRUE(manifest_name && *manifest_name && expected_sha && *expected_sha)
        << "Set ROBO_SELF_CONTACT_FAILURE_MANIFEST and ROBO_SELF_CONTACT_FAILURE_SHA256";
    const std::filesystem::path manifest_path(manifest_name);

    // Reuse the caller-pinned artifact validation, including source, phase,
    // geometry, profile, owner census and bounded one-pair codec admission.
    // The standalone coverage replay may remain inconclusive: production now
    // composes the native translation proof with authenticated accepted state.
    const auto replay = ReplayCandidateFailure(manifest_path, expected_sha);
    ASSERT_FALSE(replay["physics_accepted"].GetBool());
    const auto manifest_bytes = output::ReadBounded(manifest_path, FailureFixtureManifestCap);
    ASSERT_EQ(output::Sha256(manifest_bytes), expected_sha);
    const auto manifest = output::array_json::Parse(manifest_bytes, FailureFixtureManifestCap);
    const auto file = nonlinear_fixture::Read((manifest_path.parent_path() / "pair.bin").string(), false);
    ASSERT_EQ(file.pairs.size(), 1u);
    ASSERT_EQ(file.source_hash, output::array_json::UInt(manifest["source_hash"]));
    ASSERT_EQ(file.payload_hash, output::array_json::UInt(manifest["payload_hash"]));
    ASSERT_EQ(file.roster_digest, output::array_json::UInt(manifest["roster_digest"]));
    const auto& pair = file.pairs.front();
    const auto source_before = nonlinear_fixture::SourceHash(file.pairs);
    const auto roster_before = nonlinear_fixture::RosterDigest(file.pairs);

    // Endpoint equality alone cannot authorize an affine adapter. The frozen
    // native per-member coefficients must certify exactly zero curvature.
    for (unsigned side = 0; side < 2; ++side) {
        ASSERT_TRUE(pair.quadratic[side].complete);
        for (const auto& vertex : pair.quadratic[side].q)
            for (const auto& component : vertex) {
                ASSERT_EQ(component.lower, 0);
                ASSERT_EQ(component.upper, 0);
            }
        ASSERT_TRUE(sct::Same(pair.accepted[side].key, pair.prepared[side].key));
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
            ASSERT_EQ(contact::fixed_triangle_features::Compare(
                          pair.accepted[side].vertex_keys[vertex], pair.prepared[side].vertex_keys[vertex]),
                      0);
            ASSERT_EQ(contact::fixed_triangle_features::Compare(
                          pair.accepted[side].edge_keys[vertex], pair.prepared[side].edge_keys[vertex]),
                      0);
        }
    }
    ASSERT_NO_FATAL_FAILURE(CheckFrozenLocalIntersection(
        pair.accepted, pair.accepted_mask, pair.accepted_intersections));
    ASSERT_NO_FATAL_FAILURE(CheckFrozenLocalIntersection(
        pair.prepared, pair.prepared_mask, pair.prepared_intersections));

    contact::RepresentedIntervalLimits limits;
    limits.max_paths = 2;
    limits.max_input_pairs = 1;
    limits.max_results = 1;
    limits.max_work_per_pair = output::array_json::UInt(manifest["crossing_work"]);
    limits.max_total_work = limits.max_work_per_pair;
    limits.max_depth = static_cast<unsigned>(output::array_json::UInt(manifest["crossing_depth"]));
    contact::RepresentedIntervalCrossing crossing;
    ASSERT_EQ(crossing.Initialize(limits).status, contact::RepresentedIntervalStatus::Ok);
    const contact::RepresentedTrianglePath paths[]{AffinePath(pair.accepted[0], pair.prepared[0]),
                                                  AffinePath(pair.accepted[1], pair.prepared[1])};
    const contact::RepresentedTrianglePair query{0, 1};
    ASSERT_EQ(crossing.Certify(paths, 2, &query, 1).status, contact::RepresentedIntervalStatus::Ok);
    const auto native = crossing.results();
    ASSERT_TRUE(native.complete);
    ASSERT_EQ(native.count, 1u);
    ASSERT_NE(native.data, nullptr);
    const auto raw = native.data[0];
    ASSERT_TRUE(contact::HasExactCommonTranslationProof(raw.geometry));
    ASSERT_EQ(raw.classification, contact::RepresentedIntervalClassification::CertifiedCrossingContact);
    ASSERT_EQ(raw.reason, contact::RepresentedIntervalReason::None);
    ASSERT_EQ(raw.work, 1u);
    ASSERT_EQ(raw.witness_time_numerator, 0u);
    ASSERT_EQ(raw.witness_time_depth, 0u);

    contact::RepresentedIntervalPairKey expected_key{{paths[0].key, paths[1].key}};
    ASSERT_LT(sct::Compare(expected_key.paths[0], expected_key.paths[1]), 0);
    ASSERT_EQ(sct::Compare(raw.key, expected_key), 0);
    auto normalized = raw;
    const contact::FixedTriangleIntersectionView intersections{
        pair.prepared_intersections.data(), pair.prepared_intersections.size(), true};
    ASSERT_EQ(sct::NormalizeExactTranslatedLocal(intersections, &normalized),
              sct::TranslatedLocalStatus::Certified);
    ASSERT_EQ(normalized.geometry, contact::RepresentedIntersectionGeometry::CertifiedLocalTopology);
    ASSERT_EQ(normalized.feature.kind, contact::RepresentedFeatureKind::TriangleIntersection);
    ASSERT_EQ(normalized.accepted_event, SIZE_MAX);
    ASSERT_EQ(normalized.work, raw.work);
    ASSERT_EQ(sct::Compare(normalized.key, raw.key), 0);

    contact::SelfContactCandidatePolicyOutcome outcome;
    std::size_t count = 0;
    sct::CandidateValidationInput input;
    input.canonical_pairs = &expected_key;
    input.pair_count = 1;
    input.features = {pair.prepared_features.data(), pair.prepared_features.size(), true};
    input.intersections = intersections;
    input.crossings = {&normalized, 1, true};
    input.accepted_events = pair.accepted_owners.data();
    input.accepted_event_count = pair.accepted_owners.size();
    input.outcomes = &outcome;
    input.outcome_capacity = 1;
    input.outcome_count = &count;
    ASSERT_EQ(sct::ValidateCandidatePublications(input).status, contact::SelfContactTransactionStatus::Ok);
    ASSERT_EQ(count, 1u);
    EXPECT_EQ(outcome.disposition, contact::SelfContactCandidateDisposition::ExcludedLocalIntersection);
    EXPECT_EQ(outcome.accepted_event, SIZE_MAX);
    EXPECT_EQ(outcome.source_order, UINT64_MAX);
    EXPECT_EQ(sct::Compare(outcome.pair, expected_key), 0);
    EXPECT_EQ(nonlinear_fixture::SourceHash(file.pairs), source_before);
    EXPECT_EQ(nonlinear_fixture::RosterDigest(file.pairs), roster_before);
    RecordProperty("manifest_sha256", expected_sha);
    RecordProperty("scope", "frozen native geometry and publication replay; no physical acceptance or commit");
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
