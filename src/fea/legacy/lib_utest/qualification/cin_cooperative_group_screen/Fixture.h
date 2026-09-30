// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_parallel_groups/Fixture.h"
#include "lib_src/solvers/cin_advance/GroupScreenValues.h"
#include "FrozenGroups.h"
#include "FrozenScreen.h"
#include "FrozenCapture.h"
#include "lib_src/solvers/cin_limiter/Capture.h"
#include <cstring>
#include <gtest/gtest.h>

namespace tl::fea::cooperative_test {
namespace packet = cin_parallel_test;
namespace staged = cin_advance::group_screen;
namespace groups = cin_advance::groups;
namespace screen = cin_advance::screen;
inline void Exact(double a, double b) {
  std::uint64_t x, y;
  std::memcpy(&x, &a, sizeof(x));
  std::memcpy(&y, &b, sizeof(y));
  EXPECT_EQ(x, y) << a << " versus " << b;
}
inline void Same(const groups::Report& a, const groups::Report& b) {
  Exact(a.minimum_dt, b.minimum_dt);
  EXPECT_EQ(a.first_node, b.first_node);
  EXPECT_EQ(a.last_node, b.last_node);
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.visited, b.visited);
  EXPECT_EQ(a.bounded, b.bounded);
}
inline packet::Packet Members(std::initializer_list<unsigned> counts) {
  packet::Packet p(false, true);
  cin_group_test::PopulateGroups(p, counts.size());
  unsigned total = 0;
  for (auto count : counts) total += count;
  EXPECT_LE(total, 260u);
  p.members.assign(total, {});
  p.member_nodes.assign(packet::Nodes, 0);
  unsigned group = 0, offset = 0;
  for (auto count : counts) {
    auto& range = p.groups[group];
    range.offset = offset;
    range.count = count;
    NodalRigidGroupState state;
    state.center = {10000+double(group), -.25, .125};
    state.omega = {.001, -.002, .003};
    state.principal_axes = {{1,0,0,0,1,0,0,0,1}};
    rigid::WriteGroupState(p.accepted.data()+19*packet::Nodes+rigid::GroupStateValues*group, state);
    for (unsigned local = 0; local < count; ++local) {
      const auto node = packet::Nodes-total+offset+local;
      p.members[offset+local] = {node, 1, .1};
      p.member_nodes[node] = group%2 ? rigid::PhysicalPlainMemberNode : rigid::PartMemberNode;
      p.fixed[packet::Nodes+node] = p.fixed[2*packet::Nodes+node] = 0;
      p.present[node] = 1;
      p.accepted[p.TailOffset()+node] = 1;
      p.accepted[p.TailOffset()+packet::Nodes+node] = .1;
      p.accepted[3*node] = state.center.x+.03125*(int(local%7)-3);
      p.accepted[3*node+1] = state.center.y+.015625*(int(local%11)-5);
      p.accepted[3*node+2] = state.center.z+.0625*(int(local%5)-2);
    }
    ++group;
    offset += count;
  }
  p.Begin(1);
  return p;
}
inline groups::Report Tiled(const cin_timestep::Sources& source, double factor, unsigned group) {
  auto state = staged::BeginGroup(source, group);
  if (!state.stopped) {
    const auto body = staged::ReadBody(state.body);
    const auto range = source.rigid.groups[group];
    for (unsigned first = 0; first < range.count && !state.stopped;) {
      const auto count = std::min<unsigned>(staged::Threads, range.count-first);
      staged::MemberResponses values[staged::Threads];
      // Deliberately complete independent workers in reverse order.
      for (unsigned local = count; local-- > 0;)
        values[local] = staged::PrepareMember(source, body, range.offset+first+local);
      for (unsigned local = 0; local < count; ++local)
        staged::FoldMember(source, body, range.offset+first+local, values[local], state);
      first += count;
    }
    staged::FinishGroup(factor, state);
  }
  return state.report;
}
inline void Compare(packet::Packet& p, bool expected = true) {
  const auto input = p.Input();
  const auto source = screen::Sources(input);
  std::vector<groups::Report> reports(p.groups.size());
  for (unsigned g = 0; g < p.groups.size(); ++g) {
    reports[g] = Tiled(source, .8, g);
    Same(reports[g], frozen_groups::ScreenGroup(source, .8, g));
    Same(reports[g], groups::ScreenGroup(source, .8, g));
  }
  auto summary = screen::Empty();
  for (unsigned n = 0; n < source.nodes; ++n) screen::Observe(source, .8, n, summary);
  const cin_timestep::Result seed{12, 13, 14, true};
  auto a = seed, b = seed;
  unsigned na = 81, nb = 81;
  const bool valid = frozen::Screen(source, .8, a, na);
  EXPECT_EQ(valid, expected);
  EXPECT_EQ(valid, groups::CompleteScreen(source, summary, reports.data(), b, nb));
  Exact(a.minimum_dt, b.minimum_dt);
  EXPECT_EQ(a.limiting_node, b.limiting_node);
  EXPECT_EQ(a.limiting_group, b.limiting_group);
  EXPECT_EQ(a.valid, b.valid);
  EXPECT_EQ(na, nb);
  if (!valid) Exact(a.minimum_dt, seed.minimum_dt);
  const auto old_witness = frozen_limiter::Capture(source, .8, a, p.epoch, p.attempt);
  const auto new_witness = cin_limiter::Capture(source, .8, b, p.epoch, p.attempt);
  Exact(old_witness.values.trace_upper_per_s2, new_witness.values.trace_upper_per_s2);
  Exact(old_witness.values.minimum_dt_s, new_witness.values.minimum_dt_s);
  EXPECT_EQ(old_witness.values.kind, new_witness.values.kind);
  EXPECT_EQ(old_witness.values.node, new_witness.values.node);
  EXPECT_EQ(old_witness.values.group, new_witness.values.group);
}
} // namespace tl::fea::cooperative_test
