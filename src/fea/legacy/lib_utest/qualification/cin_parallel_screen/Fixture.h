// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/cin_advance/Screen.h"
#include "lib_utest/qualification/cin_physical_timestep/Fixture.h"
#include "FrozenScreen.h"
#include <gtest/gtest.h>
#include <cstring>
#include <vector>
namespace tl::fea::cin_screen_test {
namespace dt = cin_timestep;
namespace tree = cin_advance::screen;
inline void Exact(double a, double b) {
  std::uint64_t av = 0, bv = 0;
  std::memcpy(&av, &a, sizeof(a));
  std::memcpy(&bv, &b, sizeof(b));
  EXPECT_EQ(av, bv);
}
inline void Same(const dt::Result& a, const dt::Result& b) {
  Exact(a.minimum_dt, b.minimum_dt);
  EXPECT_EQ(a.limiting_node, b.limiting_node);
  EXPECT_EQ(a.limiting_group, b.limiting_group);
  EXPECT_EQ(a.valid, b.valid);
}
struct Fixture {
  std::uint32_t nodes;
  std::vector<double> accepted, mass, inertia, translation, rotation;
  std::vector<std::uint8_t> fixed, fixed_rotation, present, dependent, member;
  std::vector<rigid::GroupRange> groups;
  std::vector<rigid::MemberMetric> members;
  double previous = 0;
  explicit Fixture(std::uint32_t count = 263) : nodes(count),
      accepted(19*count+36), mass(count, 2), inertia(count, 3),
      translation(count, 4), rotation(count, 6), fixed(count), fixed_rotation(count),
      present(count, 1), dependent(count), member(count) {
    cin_step_test::Fixture tiny;
    EXPECT_GE(count, tiny.N);
    std::copy(tiny.accepted.begin(), tiny.accepted.begin()+19*tiny.N, accepted.begin());
    std::copy(tiny.accepted.begin()+19*tiny.N, tiny.accepted.end(), accepted.begin()+19*count);
    std::copy(tiny.mass.begin(), tiny.mass.end(), mass.begin());
    std::copy(tiny.inertia.begin(), tiny.inertia.end(), inertia.begin());
    std::copy(tiny.translation.begin(), tiny.translation.end(), translation.begin());
    std::copy(tiny.rotation.begin(), tiny.rotation.end(), rotation.begin());
    std::copy(tiny.fixed.begin(), tiny.fixed.end(), fixed.begin());
    std::copy(tiny.fixed_rotation.begin(), tiny.fixed_rotation.end(), fixed_rotation.begin());
    std::copy(tiny.present.begin(), tiny.present.end(), present.begin());
    std::copy(tiny.secondary.begin(), tiny.secondary.end(), dependent.begin());
    std::copy(tiny.member.begin(), tiny.member.end(), member.begin());
    groups.assign(tiny.groups.begin(), tiny.groups.end());
    members.assign(tiny.members.begin(), tiny.members.end());
  }
  dt::Sources View() const {
    return {accepted.data(), mass.data(), inertia.data(), translation.data(), rotation.data(),
      fixed.data(), fixed_rotation.data(), present.data(), dependent.data(),
      {groups.data(), members.data(), member.data(), std::uint32_t(groups.size()),
        std::uint32_t(members.size()), .001}, nodes, previous};
  }
};
inline bool Frozen(const dt::Sources& s, double factor, dt::Result& output, std::uint32_t& invalid) {
  const cin_screen_frozen::Sources source{s.accepted, s.mass, s.inertia, s.translation, s.rotation,
      s.fixed_translation, s.fixed_rotation, s.rotation_present, s.cin_secondary,
      s.rigid, s.nodes, s.previous_drift_dt};
  cin_screen_frozen::Result result{output.minimum_dt, output.limiting_node, output.limiting_group, output.valid};
  const bool success = cin_screen_frozen::Screen(source, factor, result, invalid);
  output = {result.minimum_dt, result.limiting_node, result.limiting_group, result.valid};
  return success;
}
inline tree::Summary Reduce(const dt::Sources& source, double factor) {
  const auto blocks = tree::Blocks(source.nodes);
  std::vector<tree::Summary> partial(blocks);
  std::array<tree::Summary, tree::Threads> lane;
  auto fold = [&] {
    for (unsigned offset = tree::Threads/2; offset; offset /= 2)
      for (unsigned t = 0; t < offset; ++t) tree::Merge(lane[t], lane[t+offset]);
  };
  for (unsigned b = 0; b < blocks; ++b) {
    lane.fill(tree::Empty());
    for (unsigned t = 0; t < tree::Threads; ++t)
      for (unsigned n = b*tree::Threads+t; n < source.nodes; n += blocks*tree::Threads)
        tree::Observe(source, factor, n, lane[t]);
    fold();
    partial[b] = lane[0];
  }
  lane.fill(tree::Empty());
  for (unsigned t = 0; t < tree::Threads; ++t)
    for (unsigned b = t; b < blocks; b += tree::Threads) tree::Merge(lane[t], partial[b]);
  fold();
  return lane[0];
}
inline bool Staged(const dt::Sources& source, double factor, dt::Result& output, std::uint32_t& invalid) {
  if (!dt::detail::CheckSources(source, factor)) return false;
  return tree::Complete(source, factor, Reduce(source, factor), output, invalid);
}
inline void Compare(const dt::Sources& source, double factor, bool expected,
    std::uint32_t expected_invalid = UINT32_MAX) {
  const dt::Result seed{19.25, 123, 456, true};
  auto old = seed, serial = seed, parallel = seed;
  std::uint32_t a = 87, b = 87, c = 87;
  EXPECT_EQ(Frozen(source, factor, old, a), expected);
  EXPECT_EQ(dt::Screen(source, factor, serial, b), expected);
  EXPECT_EQ(Staged(source, factor, parallel, c), expected);
  Same(serial, old);
  Same(parallel, old);
  EXPECT_EQ(a, expected_invalid);
  EXPECT_EQ(b, a);
  EXPECT_EQ(c, a);
  if (!expected) Same(old, seed);
}
} // namespace tl::fea::cin_screen_test
