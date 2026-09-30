#include "Packet.h"
#include <gtest/gtest.h>
#include <cstring>
namespace solid_parallel_test {
TEST(SolidParallelAssemblyHost, IncidenceRetainsEveryFamilyParentAndRepeatedSlot) {
  Packet p;
  auto& state = p.State();
  std::vector<bool> seen(state.assembly.occurrences);
  for (std::size_t node = 0; node < p.nodes; ++node) {
    std::uint32_t prior = 0;
    for (auto i = state.assembly.offsets[node]; i < state.assembly.offsets[node+1]; ++i) {
      const auto ordinal = state.assembly.incidence[i];
      ASSERT_LT(ordinal, seen.size());
      EXPECT_FALSE(seen[ordinal]);
      seen[ordinal] = true;
      if (i != state.assembly.offsets[node]) EXPECT_GT(ordinal, prior);
      prior = ordinal;
      d::AssemblyOccurrence value;
      ASSERT_TRUE(d::ReadAssemblyOccurrence<false>(state, 0, ordinal, value));
      EXPECT_EQ(value.node, node);
    }
  }
  for (const auto present : seen) EXPECT_TRUE(present);
  EXPECT_EQ(seen.size(), 2u*(8+8+6+8+8));
}
TEST(SolidParallelAssemblyHost, ExactArenaCapIncludesIndexAndAllNodeScratch) {
  Packet p;
  EXPECT_EQ(p.layout.assembly_offsets.count, p.nodes+1);
  EXPECT_EQ(p.layout.assembly_incidence.count, 76u);
  EXPECT_EQ(p.layout.assembly_nodes.count, p.nodes);
  EXPECT_EQ(sizeof(d::AssemblyNode), 4*sizeof(double));
  auto config = p.State().config;
  const d::Counts counts{2,2,2,1,1,1,2,2,1,1};
  d::ArenaLayout result;
  config.limits.max_device_bytes = p.layout.bytes-1;
  EXPECT_FALSE(d::MakeLayout(counts, config, result));
  EXPECT_EQ(result.bytes, 0u);
  config.limits.max_device_bytes = p.layout.bytes;
  EXPECT_TRUE(d::MakeLayout(counts, config, result));
  EXPECT_EQ(result.bytes, p.layout.bytes);
}
TEST(SolidParallelAssemblyHost, OrderedBuilderRejectsBeforeAnyOutputAndKeepsRepeatedUses) {
  std::uint32_t offsets[4]{7,7,7,7}, incidence[4]{9,9,9,9};
  std::size_t nodes[]{1,1,2,3};
  EXPECT_FALSE(tl::util::BuildOrderedNodeIncidence<1>(4,3,
      [&](auto p, auto){return nodes[p];}, offsets,4,incidence,4));
  for (auto x:offsets) EXPECT_EQ(x,7u);
  for (auto x:incidence) EXPECT_EQ(x,9u);
  nodes[3]=1;
  ASSERT_TRUE(tl::util::BuildOrderedNodeIncidence<1>(4,3,
      [&](auto p, auto){return nodes[p];}, offsets,4,incidence,4));
  EXPECT_EQ(offsets[1],0u); EXPECT_EQ(offsets[2],3u);
  EXPECT_EQ(incidence[0],0u); EXPECT_EQ(incidence[1],1u); EXPECT_EQ(incidence[2],3u);
}
} // namespace solid_parallel_test
