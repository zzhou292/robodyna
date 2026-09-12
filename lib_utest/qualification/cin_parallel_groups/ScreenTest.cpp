// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "FrozenValues.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace tl::fea::cin_group_test {
namespace groups = cin_advance::groups;
namespace screen = cin_advance::screen;
namespace dt = cin_timestep;
void Exact(double a, double b) {
  std::uint64_t x, y;
  std::memcpy(&x, &a, 8); std::memcpy(&y, &b, 8);
  EXPECT_EQ(x, y);
}
void Same(const dt::Result& a, const dt::Result& b) {
  Exact(a.minimum_dt, b.minimum_dt);
  EXPECT_EQ(a.limiting_node, b.limiting_node);
  EXPECT_EQ(a.limiting_group, b.limiting_group);
  EXPECT_EQ(a.valid, b.valid);
}
void Compare(packet::Packet& p, bool expected, unsigned expected_node = UINT32_MAX) {
  auto input = p.Input();
  const auto source = screen::Sources(input);
  auto summary = screen::Empty();
  for (unsigned n = 0; n < source.nodes; ++n) screen::Observe(source, .8, n, summary);
  std::vector<groups::Report> reports(p.groups.size());
  for (std::size_t g = p.groups.size(); g-- > 0;) reports[g] = groups::ScreenGroup(source, .8, g);
  const dt::Result seed{19.25, 123, 456, true};
  auto old = seed, current = seed, staged = seed;
  unsigned a = 71, b = 71, c = 71;
  EXPECT_EQ(frozen_values::Screen(source, .8, old, a), expected);
  EXPECT_EQ(dt::Screen(source, .8, current, b), expected);
  EXPECT_EQ(groups::CompleteScreen(source, summary, reports.data(), staged, c), expected);
  Same(old, current); Same(old, staged);
  EXPECT_EQ(a, expected_node); EXPECT_EQ(a, b); EXPECT_EQ(a, c);
  if (!expected) Same(old, seed);
}
TEST(CinParallelGroupsScreen, ExactLimitsUnboundedAndMemberOrderAcrossBlockBoundary) {
  for (unsigned count : {0, 1, 2, 64, 65, 129}) {
    packet::Packet p(false, false);
    PopulateGroups(p, count);
    Compare(p, true);
    ReverseGroups(p);
    Compare(p, true);
    std::fill(p.work.begin(), p.work.begin()+2*packet::Nodes, 0);
    Compare(p, true);
  }
}
TEST(CinParallelGroupsScreen, MalformedRangeCarriesLastVisitedNodeAndOrdinaryWins) {
  packet::Packet p(true, false);
  p.groups[1].count = 1;
  Compare(p, false, p.members[2].node);
  p.groups[0].count = 1;
  Compare(p, false, packet::Nodes-1);
  p.work[127] = -1;
  Compare(p, false, 127);
  p = packet::Packet(true, false);
  ReverseGroups(p);
  p.groups[0].mass = 0;
  p.groups[1].mass = 0;
  Compare(p, false, p.members[p.groups[0].offset].node);
  p = packet::Packet(true, false);
  p.members[1].node = packet::Nodes;
  Compare(p, false, packet::Nodes);
}
TEST(CinParallelGroupsScreen, StrictTieAndOutputAtomicityDoNotDependOnCompletionOrder) {
  packet::Packet p(true, false);
  auto input = p.Input();
  const auto source = screen::Sources(input);
  groups::Report reports[]{{.25, 9, 11, NodalStatus::Ok, true, true},
      {.25, 3, 4, NodalStatus::Ok, true, true}};
  dt::Result out;
  unsigned invalid = 0;
  ASSERT_TRUE(groups::CompleteScreen(source, {.25, 7, UINT32_MAX}, reports, out, invalid));
  EXPECT_EQ(out.limiting_node, 7u); EXPECT_EQ(out.limiting_group, UINT32_MAX);
  ASSERT_TRUE(groups::CompleteScreen(source, screen::Empty(), reports, out, invalid));
  EXPECT_EQ(out.limiting_node, 9u); EXPECT_EQ(out.limiting_group, 0u);
  const auto prior = out;
  reports[1].status = NodalStatus::InvalidOutput;
  ASSERT_FALSE(groups::CompleteScreen(source, screen::Empty(), reports, out, invalid));
  Same(prior, out); EXPECT_EQ(invalid, 4u);
}
} // namespace tl::fea::cin_group_test
