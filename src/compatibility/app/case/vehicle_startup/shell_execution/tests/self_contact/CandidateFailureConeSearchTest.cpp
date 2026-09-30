#include "CandidateFailureConeAssertions.h"
#include "lib_utest/qualification/self_contact_transaction/ConeDirectionOracle.h"

#include <cstdlib>

namespace crash::cases::vehicle_startup::shell_execution::self_contact_test {
namespace {
constexpr const char* ManifestSha =
    "f9f22f09f005775a2d23a8289eeb314a2043546fb4ca65a1eba81b8a695b6053";
constexpr const char* PairSha =
    "ee5aa0f90a24b74bf733c70863403f164e85aa26222179972f84640b259bb09d";

void InspectEpochOneAndIndependentGeometry(const nonlinear_fixture::File& file,
                                         const output::Value& manifest) {
    const auto& pair = file.pairs.front();
    EXPECT_EQ(file.phase.accepted_epoch, 1u);
    EXPECT_EQ(file.phase.prepared_base_epoch, 1u);
    EXPECT_EQ(file.phase.accepted_time_bits, output::Bits(2e-7));
    EXPECT_EQ(file.phase.prepared_base_time_bits, output::Bits(2e-7));
    EXPECT_EQ(file.phase.prepared_time_bits, output::Bits(4e-7));
    EXPECT_EQ(output::array_json::UInt(manifest["accepted_epoch"]), 1u);
    EXPECT_EQ(output::array_json::UInt(manifest["attempt"]), 3u);
    EXPECT_EQ(output::array_json::UInt(manifest["full_owner_count"]), 32488u);
    EXPECT_EQ(output::array_json::UInt(manifest["compact_owner_count"]), 0u);
    EXPECT_EQ(output::array_json::UInt(manifest["crossing_work"]), 4095u);
    EXPECT_EQ(output::array_json::UInt(manifest["crossing_depth"]), 20u);
    ASSERT_TRUE(manifest["owners_equivalent"].GetBool());
    // Preserve observed failure and standalone replay provenance; baseline
    // nonlinear fields in the blob were unreported, not measured zero work.
    EXPECT_EQ(pair.baseline_status, sct::NonlinearSeparationStatus::InvalidInput);
    EXPECT_EQ(pair.baseline_work, 0u);
    for (const char* capture : {"compact_replay", "full_ledger_replay"}) {
        const auto& policy = manifest[capture]["policy"];
        EXPECT_EQ(policy["status_code"].GetUint(),
                  unsigned(sct::NonlinearSeparationStatus::MissingAcceptedOwner));
        EXPECT_EQ(output::array_json::UInt(policy["work"]), 51u);
        EXPECT_EQ(output::array_json::UInt(policy["deepest"]), 20u);
        EXPECT_TRUE(policy["depth_exhausted"].GetBool());
    }
    for (unsigned side = 0; side < 2; ++side) {
        ASSERT_EQ(output::Bits(pair.half_thickness[side]), 4563130729260870543ull);
        ASSERT_GT(pair.half_thickness[side], 0);
    }
    // Independent, unlimited rational arithmetic over the actual binary64
    // coordinate differences. This rounded axis is test evidence only; the
    // production search must derive and strictly verify its own generic axis.
    const contact::Vec3 axis{-0x1.ab0adbb1622d2p-1, 0x1.033b5b69501b2p-1, -0x1p+0};
    unsigned shared[2]{3, 3}, shared_count = 0;
    for (unsigned a = 0; a < 3; ++a)
        for (unsigned b = 0; b < 3; ++b)
            if (contact::fixed_triangle_features::Compare(
                    pair.accepted[0].vertex_keys[a], pair.accepted[1].vertex_keys[b]) == 0) {
                shared[0] = a; shared[1] = b; ++shared_count;
            }
    ASSERT_EQ(shared_count, 1u);
    EXPECT_EQ(pair.accepted[0].vertex_keys[shared[0]].first, 2362259u);
    for (const auto* endpoint : {pair.accepted, pair.prepared})
        for (unsigned side = 0; side < 2; ++side)
            for (unsigned vertex = 0; vertex < 3; ++vertex) {
                if (vertex == shared[side]) continue;
                const auto projection = cone_direction_test::ArmDot(
                    endpoint[side].vertices[vertex], endpoint[side].vertices[shared[side]], axis);
                if (side == 0) EXPECT_GT(projection, 0);
                else EXPECT_LT(projection, 0);
            }

    // A real nonlocal endpoint intersection: move one B arm onto A's exact
    // nonshared vertex while retaining distinct source keys and the shared
    // endpoint path. The segment from the common vertex to that point is now
    // shared geometrically, but source topology declares only a shared vertex.
    auto crossed = pair;
    const auto arm_a = (shared[0] + 1) % 3;
    const auto arm_b = (shared[1] + 1) % 3;
    crossed.prepared[1].vertices[arm_b] = crossed.prepared[0].vertices[arm_a];
    contact::FixedTriangleIntersection intersection;
    bool intersects = false;
    ASSERT_EQ(contact::fixed_triangle_features::ClassifyPairIntersection(
                  crossed.prepared[0], crossed.prepared[1], &intersection, &intersects),
              contact::FixedTriangleDiscoveryStatus::Ok);
    ASSERT_TRUE(intersects);
    ASSERT_EQ(intersection.local_exclusion, contact::FixedTriangleLocalExclusion::None);
    ASSERT_TRUE(contact::RequiresIntersectionAdmission(intersection));
    const auto policy = sct::CertifyQuadraticFacetPolicyCoverage(
        crossed.accepted[0], crossed.prepared[0], crossed.quadratic[0], crossed.half_thickness[0],
        crossed.accepted[1], crossed.prepared[1], crossed.quadratic[1], crossed.half_thickness[1],
        2e-7, nullptr, 0, nullptr, 0, 4095, 20);
    EXPECT_FALSE(prepared_replay::Certified(policy.status));
    EXPECT_NE(policy.status, sct::NonlinearSeparationStatus::CertifiedLocalIntersection);
}
}  // namespace

TEST(CandidateFailureConeSearchReplay, EpochOneAffinePairNeedsGenericSearchAndCompleteLocalPolicy) {
    const auto* path = std::getenv("ROBO_SELF_CONTACT_CONE_SEARCH_FAILURE_MANIFEST");
    ASSERT_TRUE(path && *path)
        << "Set ROBO_SELF_CONTACT_CONE_SEARCH_FAILURE_MANIFEST to gate12 failure.json";
    cone_replay::CheckPinnedPolicy(path, ManifestSha, PairSha, InspectEpochOneAndIndependentGeometry);
}
}  // namespace crash::cases::vehicle_startup::shell_execution::self_contact_test
