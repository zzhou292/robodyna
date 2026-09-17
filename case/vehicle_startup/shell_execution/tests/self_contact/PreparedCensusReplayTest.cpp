#include "PreparedCensusReplay.h"
#include "PreparedCensusReplayManifest.h"
#include "output/BoundedArrayJson.h"
#include "lib_src/collision/FixedTriangleFeatureDiscovery.h"

#include <gtest/gtest.h>
#include <chrono>
#include <fstream>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay {
namespace {
namespace fixture = nonlinear_fixture;
namespace contact = tlfea::contact;
namespace sct = contact::self_contact_transaction;
using namespace output;

fixture::Pair SeparatedPair(std::uint64_t parent) {
    fixture::Pair pair;
    pair.baseline_status = sct::NonlinearSeparationStatus::WorkExhausted;
    pair.baseline_work = 1;
    for (unsigned side = 0; side < 2; ++side) {
        auto& triangle = pair.accepted[side];
        triangle.key = {7, parent + side, 0, 0};
        triangle.vertices[0] = {0, 0, 10.0 * side};
        triangle.vertices[1] = {1, 0, 10.0 * side};
        triangle.vertices[2] = {0, 1, 10.0 * side};
        for (unsigned vertex = 0; vertex < 3; ++vertex)
            triangle.vertex_keys[vertex] = {7, 3 * (parent + side) + vertex + 1};
        for (unsigned edge = 0; edge < 3; ++edge) {
            auto& key = triangle.edge_keys[edge]; key.parent_boundary = true;
            key.endpoints[0] = triangle.vertex_keys[edge];
            key.endpoints[1] = triangle.vertex_keys[(edge + 1) % 3];
            if (key.endpoints[1].first < key.endpoints[0].first) std::swap(key.endpoints[0], key.endpoints[1]);
        }
        pair.prepared[side] = triangle; pair.quadratic[side].complete = true;
        pair.half_thickness[side] = .001;
    }
    return pair;
}

struct Input {
    std::filesystem::path root = std::filesystem::temp_directory_path() /
        ("robo-prepared-replay-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Document manifest;
    fixture::PhaseIdentity phase;
    std::uint64_t profile_hash = 1469598103934665603ull, dt_hash = 1469598103934665603ull;
    Input() {
        Require(std::filesystem::create_directory(root), "Cannot create replay test directory");
        manifest.SetObject(); String(manifest, "schema", "robo_dyna.prepared_self_contact_census.v1");
        String(manifest, "scope", "synthetic host test evidence; not a physical run");
        Boolean(manifest, "second_interval_committed", false); Boolean(manifest, "actual_prepared_activity", true);
        Document p; p.SetObject();
        for (const auto* name : {"physical_profile", "contact_profile", "scope"}) String(p, name, "synthetic_host_fixture");
        for (const auto* name : {"leading_gap_m", "initial_speed_mps", "declared_duration_s"}) Number(p, name, 1.0);
        Number(p, "physical_step_s", 2e-7);
        for (const auto* name : {"source_instance_id", "configuration_id", "qualification_id", "self_contact_source_id",
             "event_capacity", "parent_pair_capacity", "facet_pair_capacity", "facet_chunk", "discovery_workers", "crossing_workers"})
            Integer(p, name, 1);
        Integer(p, "crossing_work_per_pair", 4095); Integer(p, "crossing_depth", 20);
        for (auto it = p.MemberBegin(); it != p.MemberEnd(); ++it) {
            fixture::HashBytes(it->name.GetString(), it->name.GetStringLength(), &profile_hash);
            if (it->value.IsString()) fixture::HashBytes(it->value.GetString(), it->value.GetStringLength(), &profile_hash);
            else if (it->value.IsUint64()) fixture::HashUnsigned(it->value.GetUint64(), &profile_hash);
            else fixture::HashUnsigned(Bits(it->value.GetDouble()), &profile_hash);
        }
        array_json::Child(manifest, "profile", p);
        phase.accepted_epoch = phase.prepared_base_epoch = 1;
        phase.accepted_time_bits = phase.prepared_base_time_bits = Bits(2e-7);
        phase.prepared_time_bits = Bits(4e-7);
        phase.accepted_velocity_time_bits = Bits(1e-7); phase.prepared_velocity_time_bits = Bits(3e-7);
        phase.accepted_temporal_scheme = phase.prepared_temporal_scheme = 1;
        phase.accepted_velocity_phase = phase.prepared_velocity_phase = 1; phase.prepared_trajectory = 1;
        fixture::HashUnsigned(Bits(2e-7), &dt_hash); fixture::HashUnsigned(Bits(2e-7), &dt_hash);
        fixture::HashUnsigned(phase.prepared_trajectory, &dt_hash);
        for (const auto* name : {"owner_id", "accepted_epoch", "prepared_attempt", "diagnostic_host_upper_bound",
             "complete_census_digest", "inspection_digest", "selected_parents", "accepted_active_parents", "prepared_active_parents"})
            Integer(manifest, name, 1);
        Number(manifest, "accepted_time_s", 2e-7); Number(manifest, "prepared_base_time_s", 2e-7);
        Number(manifest, "prepared_time_s", 4e-7); Number(manifest, "prepared_kick_dt_s", 2e-7);
        Integer(manifest, "profile_hash", profile_hash); Integer(manifest, "dt_hash", dt_hash);
        Integer(manifest, "archive_cap_bytes", ArchiveByteCap);
        for (const auto* name : {"removing_parents", "inactive_parents", "linear_affine_pairs", "linear_swept_bounds_separated",
             "linear_prism_separated", "linear_exact_geometry_pairs", "linear_common_translation", "linear_residual_separated",
             "linear_persistent_accepted", "linear_represented_pairs", "linear_represented_separated", "linear_represented_crossing",
             "linear_represented_degenerate", "linear_work_exhausted_pairs", "linear_represented_arithmetic_range", "linear_represented_work",
             "nonlinear_pairs", "nonlinear_initial_separated", "nonlinear_initial_unresolved", "nonlinear_exact_local",
             "nonlinear_residual_separated", "nonlinear_persistent_accepted", "nonlinear_remaining", "nonlinear_fixture_pairs", "fixture_bytes"})
            Integer(manifest, name, 0);
        Boolean(manifest, "linear_complete", true); Boolean(manifest, "linear_roster_complete", true);
        String(manifest, "linear_crossing_status_scope", "successful complete census; no per-pair engine status in retained roster");
        String(manifest, "linear_fixture_depth_scope", "baseline_depth zero means unknown; profile records configured crossing_depth");
        String(manifest, "nonlinear_exact_local_scope", "endpoint-only descriptive classification; all such rows retained for continuous replay");
        Integer(manifest, "linear_fixture_crossing_classification", static_cast<unsigned>(contact::RepresentedIntervalClassification::Unresolved));
        Integer(manifest, "linear_fixture_crossing_reason", static_cast<unsigned>(contact::RepresentedIntervalReason::WorkExhausted));
        manifest.AddMember("files", Value(rapidjson::kArrayType), manifest.GetAllocator());
    }
    ~Input() { std::error_code ignored; std::filesystem::remove_all(root, ignored); }
    void Append(std::uint64_t parent, fixture::PhaseIdentity* override_phase = nullptr,
                const fixture::Pair* override_pair = nullptr) {
        const auto index = manifest["files"].Size();
        const std::string file = "linear-" + std::to_string(index) + ".bin";
        fixture::Write((root / file).string(), {override_pair ? *override_pair : SeparatedPair(parent)}, profile_hash, dt_hash, 1,
            override_phase ? *override_phase : phase, false);
        const auto decoded = fixture::Read((root / file).string(), false);
        const auto bytes = ReadBounded(root / file, ShardByteCap);
        Document item; item.SetObject(); String(item, "file", file); String(item, "sha256", Sha256(bytes));
        String(item, "family", "linear"); Integer(item, "bytes", bytes.size()); Integer(item, "pairs", 1);
        Integer(item, "source_hash", decoded.source_hash); Integer(item, "payload_hash", decoded.payload_hash);
        Integer(item, "roster_digest", decoded.roster_digest);
        Value entry; entry.CopyFrom(item, manifest.GetAllocator()); manifest["files"].PushBack(entry, manifest.GetAllocator());
        for (const auto* name : {"linear_affine_pairs", "linear_exact_geometry_pairs", "linear_represented_pairs", "linear_work_exhausted_pairs"})
            manifest[name].SetUint64(index + 1);
        manifest["fixture_bytes"].SetUint64(manifest["fixture_bytes"].GetUint64() + bytes.size());
    }
    std::string Seal() {
        WriteJson(root / "census.json", manifest);
        return Sha256(ReadBounded(root / "census.json", ManifestByteCap));
    }
};
}

TEST(PreparedCensusReplay, VisitsAllShardsAndUsesActualPolicyProof) {
    Input input; input.Append(100); input.Append(200); const auto hash = input.Seal();
    std::size_t observed = 0;
    const auto summary = Replay(input.root / "census.json", hash, [&](const PairResult& pair) {
        EXPECT_EQ(pair.facets[0].parent_eid, observed ? 200u : 100u); EXPECT_TRUE(pair.affine);
        EXPECT_EQ(pair.policy.status, sct::NonlinearSeparationStatus::CertifiedSeparated); ++observed;
    });
    EXPECT_TRUE(summary.complete); EXPECT_EQ(summary.files, 2u); EXPECT_EQ(summary.pairs, 2u);
    EXPECT_EQ(observed, 2u); EXPECT_EQ(summary.noncertified, 0u);
    EXPECT_EQ(summary.result_digest, Replay(input.root / "census.json", hash, {}).result_digest);
}

TEST(PreparedCensusReplay, NoncertifiedPairIsReportedWithoutDroppingLaterPairs) {
    Input input; input.Append(100);
    auto contact_pair = SeparatedPair(200);
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
        contact_pair.accepted[1].vertices[vertex].z = .001;
        contact_pair.prepared[1].vertices[vertex].z = .001;
    }
    input.Append(200, nullptr, &contact_pair); input.Append(300);
    std::size_t seen = 0, failed = 0;
    const auto result = Replay(input.root / "census.json", input.Seal(), [&](const PairResult& pair) {
        ++seen;
        if (!Certified(pair.policy.status)) {
            ++failed; EXPECT_EQ(pair.facets[0].parent_eid, 200u);
            EXPECT_EQ(PairDocument(pair)["first_facet"]["parent_eid"].GetUint64(), 200u);
        }
    });
    EXPECT_EQ(seen, 3u); EXPECT_EQ(failed, 1u); EXPECT_EQ(result.noncertified, 1u); EXPECT_TRUE(result.complete);
}

TEST(PreparedCensusReplay, LocalTopologyUsesContinuousProofAndKeepsLedgerResultSeparate) {
    Input input;
    auto pair = SeparatedPair(100);
    // Reuse the analytically separated shared-vertex geometry from the native
    // continuous-local coupon. This tests codec/policy reporting, not an
    // additional unresolved geometric class.
    pair.accepted[0].vertices[1] = {1, 0, 0};
    pair.accepted[0].vertices[2] = {1, 1, 0};
    pair.accepted[1].vertices[0] = {0, 0, 0};
    pair.accepted[1].vertices[1] = {0, -1, 1};
    pair.accepted[1].vertices[2] = {0, -1, -1};
    pair.accepted[1].vertex_keys[0] = pair.accepted[0].vertex_keys[0];
    for (auto& triangle : pair.accepted) {
        for (unsigned edge = 0; edge < 3; ++edge) {
            auto& key = triangle.edge_keys[edge];
            key.endpoints[0] = triangle.vertex_keys[edge];
            key.endpoints[1] = triangle.vertex_keys[(edge + 1) % 3];
            if (contact::fixed_triangle_features::Compare(key.endpoints[1], key.endpoints[0]) < 0)
                std::swap(key.endpoints[0], key.endpoints[1]);
        }
    }
    pair.prepared[0] = pair.accepted[0]; pair.prepared[1] = pair.accepted[1];
    input.Append(100, nullptr, &pair);
    const auto summary = Replay(input.root / "census.json", input.Seal(), [](const PairResult& result) {
        EXPECT_EQ(result.policy.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
        EXPECT_NE(result.ledger.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
        EXPECT_EQ(result.owners, 0u); EXPECT_EQ(result.exclusions, 0u);
        const auto report = PairDocument(result);
        EXPECT_STREQ(report["policy"]["status"].GetString(), "certified_local_intersection");
        EXPECT_TRUE(report["policy"]["certified"].GetBool());
    });
    EXPECT_TRUE(summary.complete); EXPECT_EQ(summary.noncertified, 0u);
    EXPECT_EQ(summary.status_counts[static_cast<unsigned>(sct::NonlinearSeparationStatus::CertifiedLocalIntersection)], 1u);
}

TEST(PreparedCensusReplay, HistoricalPersistenceOmissionsRemainVisibleWithoutClaimingGeometry) {
    Input input;
    for (const auto* name : {"linear_affine_pairs", "linear_exact_geometry_pairs", "linear_persistent_accepted"})
        input.manifest[name].SetUint64(1);
    input.manifest["nonlinear_pairs"].SetUint64(1);
    input.manifest["nonlinear_initial_unresolved"].SetUint64(1);
    input.manifest["nonlinear_persistent_accepted"].SetUint64(1);
    const auto hash = input.Seal();
    const auto summary = Replay(input.root / "census.json", hash, {});
    EXPECT_TRUE(summary.complete); EXPECT_EQ(summary.pairs, 0u);
    EXPECT_EQ(summary.omitted_nonlinear_persistent, 1u);
    EXPECT_EQ(summary.omitted_linear_persistent, 1u);
    const auto report = SummaryDocument(summary, hash);
    EXPECT_EQ(report["omitted_nonlinear_persistent_pairs"].GetUint64(), 1u);
    EXPECT_EQ(report["omitted_linear_persistent_pairs"].GetUint64(), 1u);
}

TEST(PreparedCensusReplay, RetainedPersistenceMetadataStillRequiresActualPolicyProof) {
    Input input;
    auto pair = SeparatedPair(100);
    for (unsigned vertex = 0; vertex < 3; ++vertex) {
        pair.accepted[1].vertices[vertex].z = .001;
        pair.prepared[1].vertices[vertex].z = .001;
    }
    pair.quadratic[1].q[0][2] = {1, 1};
    input.Append(100, nullptr, &pair);
    input.manifest["files"][0]["family"].SetString("nonlinear", input.manifest.GetAllocator());
    for (const auto* name : {"linear_affine_pairs", "linear_exact_geometry_pairs", "linear_represented_pairs", "linear_work_exhausted_pairs"})
        input.manifest[name].SetUint64(0);
    for (const auto* name : {"nonlinear_pairs", "nonlinear_initial_unresolved", "nonlinear_persistent_accepted", "nonlinear_fixture_pairs"})
        input.manifest[name].SetUint64(1);
    const auto hash = input.Seal();
    std::size_t visited = 0;
    const auto summary = Replay(input.root / "census.json", hash, [&](const PairResult& result) {
        ++visited;
        EXPECT_EQ(result.family, "nonlinear"); EXPECT_FALSE(result.affine);
        // This synthetic source classification supplies no real owner. Merely
        // calling a row persistent must neither omit it nor certify contact.
        EXPECT_EQ(result.owners, 0u);
        EXPECT_FALSE(Certified(result.policy.status));
    });
    EXPECT_TRUE(summary.complete); EXPECT_EQ(visited, 1u);
    EXPECT_EQ(summary.pairs, 1u); EXPECT_EQ(summary.nonlinear, 1u);
    EXPECT_EQ(summary.noncertified, 1u);
    EXPECT_EQ(summary.omitted_nonlinear_persistent, 0u);
    EXPECT_EQ(summary.omitted_linear_persistent, 0u);
    const auto report = SummaryDocument(summary, hash);
    EXPECT_EQ(report["omitted_nonlinear_persistent_pairs"].GetUint64(), 0u);
    EXPECT_EQ(report["omitted_linear_persistent_pairs"].GetUint64(), 0u);
}

TEST(PreparedCensusReplay, RejectsWrongManifestHashMissingShardAndTraversal) {
    Input input; input.Append(100); const auto hash = input.Seal();
    EXPECT_THROW(Replay(input.root / "census.json", std::string(64, '0'), {}), std::runtime_error);
    std::filesystem::remove(input.root / "linear-0.bin");
    EXPECT_THROW(Replay(input.root / "census.json", hash, {}), std::runtime_error);
    Input traversal; traversal.Append(100);
    traversal.manifest["files"][0]["file"].SetString("../outside.bin", traversal.manifest.GetAllocator());
    EXPECT_THROW(Replay(traversal.root / "census.json", traversal.Seal(), {}), std::runtime_error);
}

TEST(PreparedCensusReplay, RejectsPhaseChangeAndDuplicateSourcePairs) {
    Input phase; auto forged = phase.phase; forged.prepared_time_bits = Bits(6e-7); phase.Append(100, &forged);
    EXPECT_THROW(Replay(phase.root / "census.json", phase.Seal(), {}), std::runtime_error);
    Input duplicate; duplicate.Append(100); duplicate.Append(100);
    EXPECT_THROW(Replay(duplicate.root / "census.json", duplicate.Seal(), {}), std::runtime_error);
}

TEST(PreparedCensusReplay, RejectsPayloadTamperingReorderedInventoryAndOversizedShard) {
    Input corrupt; corrupt.Append(100); const auto hash = corrupt.Seal();
    const auto path = corrupt.root / "linear-0.bin";
    { std::fstream file(path, std::ios::binary | std::ios::in | std::ios::out);
      file.seekp(209); file.put(static_cast<char>(127)); }
    EXPECT_THROW(Replay(corrupt.root / "census.json", hash, {}), std::runtime_error);
    Input reordered; reordered.Append(100); reordered.Append(200);
    reordered.manifest["files"][0].Swap(reordered.manifest["files"][1]);
    EXPECT_THROW(Replay(reordered.root / "census.json", reordered.Seal(), {}), std::runtime_error);
    Input oversized; oversized.Append(100);
    oversized.manifest["files"][0]["bytes"].SetUint64(ShardByteCap + 1);
    EXPECT_THROW(Replay(oversized.root / "census.json", oversized.Seal(), {}), std::runtime_error);
}

TEST(PreparedCensusReplay, RejectsCountPartitionAndUnboundedCodecHeaderBeforeAllocation) {
    Input count; count.Append(100); count.manifest["linear_work_exhausted_pairs"].SetUint64(2);
    EXPECT_THROW(Replay(count.root / "census.json", count.Seal(), {}), std::runtime_error);
    Input header; header.Append(100); const auto path = header.root / "linear-0.bin";
    auto bytes = ReadBounded(path, ShardByteCap);
    for (unsigned byte = 0; byte < 8; ++byte) bytes[9 * 8 + byte] = static_cast<char>(255);
    { std::ofstream out(path, std::ios::binary | std::ios::trunc); out.write(bytes.data(), bytes.size()); }
    const auto hash = Sha256(bytes);
    header.manifest["files"][0]["sha256"].SetString(hash.c_str(), header.manifest.GetAllocator());
    EXPECT_THROW(Replay(header.root / "census.json", header.Seal(), {}), std::runtime_error);
}

TEST(PreparedCensusReplay, EndpointLocalRowsCannotDisappearFromContinuousReplayInventory) {
    Input input;
    input.manifest["nonlinear_pairs"].SetUint64(1);
    input.manifest["nonlinear_initial_unresolved"].SetUint64(1);
    input.manifest["nonlinear_exact_local"].SetUint64(1);
    EXPECT_THROW(Replay(input.root / "census.json", input.Seal(), {}), std::runtime_error);
}

TEST(PreparedCensusReplay, ExclusionReuseRejectsUnauthenticatedSupportAndReportRetainsFailureFlags) {
    auto pair = SeparatedPair(100); pair.accepted_features.emplace_back();
    EXPECT_THROW(SameRigidExclusions(pair), std::runtime_error);
    pair.accepted_policy.emplace_back(); auto& policy = pair.accepted_policy.back();
    policy.pair_status = contact::SelfContactPairStatus::ExcludedSameRigidGroup;
    EXPECT_THROW(SameRigidExclusions(pair), std::runtime_error);
    policy.report_status = contact::SelfContactTransactionStatus::Ok;
    for (auto& endpoint : policy.endpoint_support) {
        endpoint.status = contact::SelfContactSupportStatus::CompleteRigidGroup; endpoint.complete_rigid_group = 4;
    }
    ASSERT_EQ(SameRigidExclusions(pair).size(), 1u);
    EXPECT_EQ(SameRigidExclusions(pair)[0].complete_rigid_group, 4u);
    PairResult result; result.policy.status = sct::NonlinearSeparationStatus::MissingAcceptedOwner;
    result.policy.depth_exhausted = true; result.policy.has_unresolved_cell = true;
    result.policy.unresolved_path = 3; result.policy.unresolved_depth = 20;
    const auto report = PairDocument(result);
    EXPECT_TRUE(report["policy"]["depth_exhausted"].GetBool());
    EXPECT_EQ(report["policy"]["unresolved_path"].GetUint64(), 3u);
    EXPECT_FALSE(report["policy"]["certified"].GetBool());
}

}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test::prepared_replay
