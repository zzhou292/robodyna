// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "lib_utils/BoundedArena.h"
namespace wall_response_test {
TEST(MappedWallResponseHost, IncidenceUsesOriginalCompactOrderAndRejectsBeforeWriting) {
  Packet p;
  for (unsigned g = 0; g < p.groups; ++g) {
    std::vector<unsigned> expected;
    for (unsigned i = 0; i < p.count; ++i) if (p.roots[i] == g) expected.push_back(i);
    const std::vector<unsigned> actual(p.rows.begin()+p.offsets[g], p.rows.begin()+p.offsets[g+1]);
    EXPECT_EQ(actual, expected);
  }
  const auto offsets = p.offsets, rows = p.rows;
  p.roots.back() = p.groups;
  EXPECT_FALSE(r::BuildIncidence(p.roots.data(), p.count, p.groups, p.Side().response));
  EXPECT_EQ(p.offsets, offsets); EXPECT_EQ(p.rows, rows);
  p.Reset();
  auto scratch = p.Side().response;
  --scratch.node_count;
  EXPECT_FALSE(r::BuildIncidence(p.roots.data(), p.count, p.groups, scratch));
  EXPECT_FALSE(r::BuildIncidence(nullptr, p.count, p.groups, p.Side().response));
  EXPECT_FALSE(r::BuildIncidence(p.roots.data(), p.count, 1025, p.Side().response));
  EXPECT_EQ(p.offsets, offsets); EXPECT_EQ(p.rows, rows);
  const auto owned = p.rows;
  std::fill(p.roots.begin(), p.roots.end(), UINT32_MAX);
  EXPECT_EQ(p.rows, owned); // No borrowed roots retained inside the CSR.
}
TEST(MappedWallResponseHost, ExactTailAndOneByteShortBudgetPreservePriorLayout) {
  for (const auto nodes : {7u, 128u, 359785u, c::MaxVehicleNodalWallDeviceNodes}) {
    m::Layout layout;
    ASSERT_TRUE(m::MakeLayout(4, nodes, 779, SIZE_MAX, layout));
    EXPECT_EQ(layout.response_offsets.count, 780u);
    EXPECT_EQ(layout.response_rows.count, nodes);
    EXPECT_EQ(layout.response_maxima.count, r::Blocks(nodes));
    EXPECT_LE(layout.response_maxima.count, r::MaximumBlocks);
    const auto old_end = layout.interval.offset+layout.interval.bytes;
    EXPECT_EQ(layout.bytes-old_end,
        4*(nodes+780u)+8*r::Blocks(nodes)+(nodes%2 ? 4u : 0u));
    m::Layout exact;
    EXPECT_TRUE(m::MakeLayout(4, nodes, 779, layout.bytes, exact));
    EXPECT_FALSE(m::MakeLayout(4, nodes, 779, layout.bytes-1, exact));
    EXPECT_EQ(exact.bytes, layout.bytes);
    RecordProperty("response_scratch_header_bytes", sizeof(r::Scratch));
    RecordProperty("response_layout_region_bytes", 3*sizeof(tl::util::ArenaRegion));
    if (nodes == 359785) RecordProperty("original_response_tail_bytes_each_arena", layout.bytes-old_end);
  }
}
} // namespace wall_response_test
