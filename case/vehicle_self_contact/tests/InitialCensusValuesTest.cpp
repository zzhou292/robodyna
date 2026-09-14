#include "../InitialCensusValues.h"

#include <gtest/gtest.h>
#include <array>

namespace crash::cases::vehicle_self_contact {
namespace {

using Key = tlfea::contact::SelfContactPairKey;

constexpr Key Pair(std::uint32_t first, std::uint32_t second) {
    return (std::uint64_t{first} << 32) | second;
}

struct Fixture {
    std::array<std::uint32_t, 4> surface_to_active{{2, 0, 3, 1}};
    std::array<InitialCensusParentRow, 4> parents{{
        {102, 1, 2, 5, {1, 4, 5, 2}, 4},
        {104, 3, 1, 7, {8, 7, 3, UINT32_MAX}, 3},
        {101, 0, 2, 5, {0, 1, 2, 3}, 4},
        {103, 2, 1, UINT32_MAX, {6, 7, 8, UINT32_MAX}, 3},
    }};
    std::array<Key, 5> keys{{
        Pair(0, 1), Pair(0, 2), Pair(0, 3),
        Pair(1, 2), Pair(2, 3)}};
};

TEST(InitialCensusValues,
     ScrambledExactMapCountsFacetClassesAndDescriptiveStaticFacts) {
    Fixture source;
    InitialFacetCapacityCensus first;
    const auto report = CountInitialFacetCapacity(
        source.keys.data(), source.keys.size(),
        source.surface_to_active.data(), source.surface_to_active.size(),
        source.parents.data(), source.parents.size(), &first);
    ASSERT_EQ(report.status, InitialCensusValueStatus::Ok)
        << report.message;
    EXPECT_EQ(first.current_inflated_aabb_overlap_parent_pairs, 5u);
    EXPECT_EQ(first.q4_q4_parent_pairs, 1u);
    EXPECT_EQ(first.q4_t3_parent_pairs, 3u);
    EXPECT_EQ(first.t3_t3_parent_pairs, 1u);
    EXPECT_EQ(first.level0_facet_pairs, 11u);
    EXPECT_EQ(first.same_complete_rigid_group_parent_pairs, 1u);
    EXPECT_EQ(first.shared_canonical_vertex_parent_pairs, 3u);
    EXPECT_EQ(first.shared_canonical_edge_parent_pairs, 2u);
    EXPECT_TRUE(first.sorted_unique_canonical_keys);
    EXPECT_TRUE(first.no_self_or_reversed_pair_keys);
    EXPECT_TRUE(first.descriptive_static_policy_only);
    EXPECT_FALSE(first.feature_discovery_performed);
    EXPECT_FALSE(first.intersection_processing_performed);
    EXPECT_FALSE(first.force_admission_performed);
    EXPECT_NE(first.pair_key_hash, 0u);
    EXPECT_NE(first.surface_active_source_hash, 0u);

    InitialFacetCapacityCensus rerun;
    ASSERT_EQ(CountInitialFacetCapacity(
        source.keys.data(), source.keys.size(),
        source.surface_to_active.data(), source.surface_to_active.size(),
        source.parents.data(), source.parents.size(), &rerun).status,
        InitialCensusValueStatus::Ok);
    EXPECT_EQ(rerun.pair_key_hash, first.pair_key_hash);
    EXPECT_EQ(rerun.surface_active_source_hash,
              first.surface_active_source_hash);
    EXPECT_EQ(rerun.level0_facet_pairs, first.level0_facet_pairs);
}

TEST(InitialCensusValues,
     InvalidKeysAndNonPermutationFailWithoutPublishingCounts) {
    Fixture source;
    InitialFacetCapacityCensus output;
    output.level0_facet_pairs = 99;
    const std::array<Key, 2> duplicate{{
        Pair(0, 1), Pair(0, 1)}};
    auto report = CountInitialFacetCapacity(
        duplicate.data(), duplicate.size(),
        source.surface_to_active.data(), source.surface_to_active.size(),
        source.parents.data(), source.parents.size(), &output);
    EXPECT_EQ(report.status, InitialCensusValueStatus::InvalidPairKey);
    EXPECT_EQ(report.pair, 1u);
    EXPECT_EQ(output.level0_facet_pairs, 99u);

    const std::array<Key, 1> reversed{{Pair(2, 1)}};
    report = CountInitialFacetCapacity(
        reversed.data(), reversed.size(),
        source.surface_to_active.data(), source.surface_to_active.size(),
        source.parents.data(), source.parents.size(), &output);
    EXPECT_EQ(report.status, InitialCensusValueStatus::InvalidPairKey);
    EXPECT_EQ(output.level0_facet_pairs, 99u);

    source.surface_to_active[0] = 0;
    report = CountInitialFacetCapacity(
        source.keys.data(), source.keys.size(),
        source.surface_to_active.data(), source.surface_to_active.size(),
        source.parents.data(), source.parents.size(), &output);
    EXPECT_EQ(report.status, InitialCensusValueStatus::IdentityMismatch);
    EXPECT_EQ(output.level0_facet_pairs, 99u);
}

}  // namespace
}  // namespace crash::cases::vehicle_self_contact
