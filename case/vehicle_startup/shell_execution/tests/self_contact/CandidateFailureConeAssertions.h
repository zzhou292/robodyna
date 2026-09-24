#pragma once

#include "CandidateFailureFixture.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"
#include "lib_src/collision/RepresentedIntervalCrossing.h"
#include "lib_src/collision/self_contact_transaction/LocalContact.h"
#include "lib_src/collision/self_contact_transaction/ResidualTasks.h"
#include "output/BoundedArrayJson.h"

#include <gtest/gtest.h>
#include <array>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::cone_replay {

inline void LocalEndpoint(const contact::CurrentFixedTriangle triangles[2],
                   std::uint16_t expected_mask,
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
    ASSERT_EQ(actual.local_exclusion, contact::FixedTriangleLocalExclusion::SharedVertexOnly);
    ASSERT_FALSE(contact::RequiresIntersectionAdmission(actual));
}

inline void CompleteLocalProof(const sct::NonlinearSeparationResult& result) {
    ASSERT_EQ(result.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
    EXPECT_EQ(result.work, 1u);
    EXPECT_EQ(result.deepest, 0u);
    EXPECT_TRUE(result.has_intersection);
    EXPECT_EQ(result.intersection_feature.kind, contact::RepresentedFeatureKind::TriangleIntersection);
    EXPECT_EQ(result.intersection_time_numerator, 0u);
    EXPECT_EQ(result.intersection_time_depth, 0u);
    EXPECT_FALSE(result.has_unresolved_cell);
    EXPECT_FALSE(result.work_exhausted);
    EXPECT_FALSE(result.depth_exhausted);
    EXPECT_EQ(result.accepted_certificate, SIZE_MAX);
    EXPECT_EQ(result.accepted_source_order, UINT64_MAX);
    EXPECT_EQ(result.covered_cells, 0u);
    EXPECT_EQ(result.closed_covered_cells, 0u);
}


// Shared gate8/gate12 assertions. Inspection is qualification-only and cannot
// provide a certificate: the ordinary native/local/policy validators below
// must each succeed independently with original positive shell thickness.
using Inspect = void (*)(const nonlinear_fixture::File&, const output::Value&);
inline void CheckPinnedPolicy(const std::filesystem::path& manifest_path,
                              const char* ManifestSha, const char* PairSha,
                              Inspect inspect = nullptr) {
    // This existing reader authenticates the complete bounded codec, phase,
    // geometry, source/profile and full-versus-compact owner evidence. A later
    // proof correction may change replay classification without changing data.
    const auto replay = ReplayCandidateFailure(manifest_path, ManifestSha);
    ASSERT_FALSE(replay["physics_accepted"].GetBool());
    const auto bytes = output::ReadBounded(manifest_path, FailureFixtureManifestCap);
    ASSERT_EQ(output::Sha256(bytes), ManifestSha);
    const auto manifest = output::array_json::Parse(bytes, FailureFixtureManifestCap);
    const auto pair_path = manifest_path.parent_path() / "pair.bin";
    ASSERT_EQ(output::Sha256(output::ReadBounded(pair_path, FailureFixtureArchiveCap)), PairSha);
    const auto file = nonlinear_fixture::Read(pair_path.string(), false);
    ASSERT_EQ(file.pairs.size(), 1u);
    ASSERT_EQ(file.source_hash, output::array_json::UInt(manifest["source_hash"]));
    ASSERT_EQ(file.payload_hash, output::array_json::UInt(manifest["payload_hash"]));
    ASSERT_EQ(file.roster_digest, output::array_json::UInt(manifest["roster_digest"]));
    const auto source_before = nonlinear_fixture::SourceHash(file.pairs);
    const auto roster_before = nonlinear_fixture::RosterDigest(file.pairs);
    const auto& pair = file.pairs.front();
    if (inspect) ASSERT_NO_FATAL_FAILURE(inspect(file, manifest));
    const auto duration = output::array_json::Real(manifest["duration_s"]);
    const auto work = output::array_json::UInt(manifest["crossing_work"]);
    const auto depth = static_cast<unsigned>(output::array_json::UInt(manifest["crossing_depth"]));
    ASSERT_EQ(output::Bits(duration), output::Bits(2e-7));
    ASSERT_EQ(manifest["native_report"]["status_code"].GetUint(),
              unsigned(contact::SelfContactTransactionStatus::UnresolvedCandidate));
    ASSERT_EQ(manifest["native_report"]["crossing_reason_code"].GetUint(),
              unsigned(contact::RepresentedIntervalReason::WorkExhausted));
    ASSERT_EQ(manifest["compact_replay"]["policy"]["status_code"].GetUint(),
              unsigned(sct::NonlinearSeparationStatus::MissingAcceptedOwner));
    ASSERT_TRUE(pair.accepted_owners.empty());
    ASSERT_EQ(pair.accepted_features.size(), 9u);
    ASSERT_EQ(pair.prepared_features.size(), 9u);

    std::array<contact::RepresentedTrianglePath, 2> paths;
    for (unsigned side = 0; side < 2; ++side) {
        ASSERT_TRUE(pair.quadratic[side].complete);
        for (const auto& vertex : pair.quadratic[side].q)
            for (const auto& component : vertex) {
                ASSERT_EQ(component.lower, 0);
                ASSERT_EQ(component.upper, 0);
            }
        ASSERT_TRUE(sct::Same(pair.accepted[side].key, pair.prepared[side].key));
        paths[side].key = {pair.prepared[side].key.source_instance_id,
            pair.prepared[side].key.parent_eid, pair.prepared[side].key.level,
            pair.prepared[side].key.local_facet};
        paths[side].motion = contact::RepresentedMotion::LinearNodalV1;
        for (unsigned vertex = 0; vertex < 3; ++vertex) {
            ASSERT_EQ(contact::fixed_triangle_features::Compare(
                pair.accepted[side].vertex_keys[vertex], pair.prepared[side].vertex_keys[vertex]), 0);
            ASSERT_EQ(contact::fixed_triangle_features::Compare(
                pair.accepted[side].edge_keys[vertex], pair.prepared[side].edge_keys[vertex]), 0);
            paths[side].vertices[vertex] = {pair.prepared[side].vertex_keys[vertex],
                {pair.accepted[side].vertices[vertex], pair.prepared[side].vertices[vertex]}};
            paths[side].edge_keys[vertex] = pair.prepared[side].edge_keys[vertex];
        }
    }
    ASSERT_NO_FATAL_FAILURE(LocalEndpoint(pair.accepted, pair.accepted_mask, pair.accepted_intersections));
    ASSERT_NO_FATAL_FAILURE(LocalEndpoint(pair.prepared, pair.prepared_mask, pair.prepared_intersections));
    const auto residual = sct::CertifyQuadraticUnmaskedSeparation(
        pair.accepted[0], pair.prepared[0], pair.quadratic[0], pair.half_thickness[0],
        pair.accepted[1], pair.prepared[1], pair.quadratic[1], pair.half_thickness[1], duration,
        {pair.prepared_features.data(), pair.prepared_features.size(), true}, {pair.prepared_mask});
    ASSERT_EQ(residual.status, sct::LinearResidualSeparationStatus::CertifiedSeparated);
    ASSERT_FALSE(residual.exact_common_translation);
    ASSERT_GT(residual.strict_gap_lower_m, 0);

    // The raw affine engine proves the existing shared-vertex intersection
    // at t=0. Candidate subsequently replaces an inconclusive whole-interval
    // policy with Unresolved/WorkExhausted: the captured final reason is not
    // the raw engine result. An ordinary first witness still needs local proof.
    contact::RepresentedIntervalLimits limits;
    limits.max_paths = 2;
    limits.max_input_pairs = 1;
    limits.max_results = 1;
    limits.max_work_per_pair = work;
    limits.max_total_work = work;
    limits.max_depth = depth;
    contact::RepresentedIntervalCrossing crossing;
    ASSERT_EQ(crossing.Initialize(limits).status, contact::RepresentedIntervalStatus::Ok);
    const contact::RepresentedTrianglePair query{0, 1};
    ASSERT_EQ(crossing.Certify(paths.data(), paths.size(), &query, 1).status,
              contact::RepresentedIntervalStatus::Ok);
    const auto raw = crossing.results();
    ASSERT_TRUE(raw.complete);
    ASSERT_EQ(raw.count, 1u);
    ASSERT_NE(raw.data, nullptr);
    ASSERT_EQ(raw.data[0].classification, contact::RepresentedIntervalClassification::CertifiedCrossingContact);
    ASSERT_EQ(raw.data[0].reason, contact::RepresentedIntervalReason::None);
    ASSERT_EQ(raw.data[0].work, 1u);
    ASSERT_EQ(raw.data[0].witness_time_numerator, 0u);
    ASSERT_EQ(raw.data[0].witness_time_depth, 0u);
    ASSERT_EQ(raw.data[0].geometry,
              pair.accepted_intersections.front().kind == contact::FixedTriangleIntersectionKind::Transverse
                  ? contact::RepresentedIntersectionGeometry::Transverse
                  : contact::RepresentedIntersectionGeometry::Coplanar);
    ASSERT_FALSE(contact::HasExactCommonTranslationProof(raw.data[0].geometry));

    const auto local = sct::CertifyQuadraticLocalContact(
        pair.accepted[0], pair.prepared[0], pair.quadratic[0], pair.half_thickness[0],
        pair.accepted[1], pair.prepared[1], pair.quadratic[1], pair.half_thickness[1], duration, work, depth);
    ASSERT_NO_FATAL_FAILURE(CompleteLocalProof(local));
    const auto exclusions = prepared_replay::SameRigidExclusions(pair);
    ASSERT_TRUE(exclusions.empty());
    const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
        pair.accepted[0], pair.prepared[0], pair.quadratic[0], pair.half_thickness[0],
        pair.accepted[1], pair.prepared[1], pair.quadratic[1], pair.half_thickness[1], duration,
        pair.accepted_owners.data(), pair.accepted_owners.size(), exclusions.data(), exclusions.size(),
        work, depth);
    ASSERT_NO_FATAL_FAILURE(CompleteLocalProof(policy));
    EXPECT_EQ(policy.proof_digest, local.proof_digest);
    ASSERT_EQ(replay["replay"]["policy"]["status_code"].GetUint(),
              unsigned(sct::NonlinearSeparationStatus::CertifiedLocalIntersection));
    EXPECT_FALSE(replay["matches_captured_compact_result"].GetBool());

    // Mirror Candidate's existing local-proof projection, then use its actual
    // final publication validator. This host check grants no live authority.
    auto normalized = raw.data[0];
    normalized.classification = contact::RepresentedIntervalClassification::CertifiedCrossingContact;
    normalized.reason = contact::RepresentedIntervalReason::None;
    normalized.feature = {};
    normalized.feature.kind = contact::RepresentedFeatureKind::TriangleIntersection;
    normalized.geometry = contact::RepresentedIntersectionGeometry::CertifiedLocalTopology;
    normalized.witness_time_numerator = 0;
    normalized.witness_time_depth = 0;
    normalized.accepted_event = SIZE_MAX;
    ASSERT_LE(policy.work, SIZE_MAX - raw.data[0].work);
    normalized.work = raw.data[0].work + policy.work;
    contact::RepresentedIntervalPairKey expected_key{{paths[0].key, paths[1].key}};
    ASSERT_LT(sct::Compare(expected_key.paths[0], expected_key.paths[1]), 0);
    ASSERT_EQ(sct::Compare(normalized.key, expected_key), 0);
    contact::SelfContactCandidatePolicyOutcome outcome;
    std::size_t count = 0;
    sct::CandidateValidationInput input;
    input.canonical_pairs = &expected_key;
    input.pair_count = 1;
    input.features = {pair.prepared_features.data(), pair.prepared_features.size(), true};
    input.intersections = {pair.prepared_intersections.data(), pair.prepared_intersections.size(), true};
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
    ::testing::Test::RecordProperty("manifest_sha256", ManifestSha);
    ::testing::Test::RecordProperty("pair_sha256", PairSha);
    ::testing::Test::RecordProperty("scope", "frozen affine local-contact and publication replay; no physical acceptance or commit");
}
}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::cone_replay
