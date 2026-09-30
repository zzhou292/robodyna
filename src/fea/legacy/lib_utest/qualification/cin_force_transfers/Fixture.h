// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_force_inputs/Fixture.h"
#include "lib_src/solvers/cin_advance/ForceTransfers.h"
#include "FrozenForce.h"

namespace tl::fea::cin_transfer_test {
namespace packet = cin_parallel_test;
namespace cin = constraints::tied_shell::cin;
namespace tied = constraints::tied_shell;
namespace frozen = constraints::tied_shell::cin_transfer_frozen;
namespace transfer = cin_advance::force_transfers;

inline packet::Packet Population(unsigned count, bool groups = false, bool capture = false) {
  packet::Packet p(groups, capture);
  const auto original = p.rows;
  p.rows.resize(count);
  p.activity.assign(count, 1);
  p.first_witness.resize(count);
  std::iota(p.first_witness.begin(), p.first_witness.end(), 0u);
  std::fill(p.dependent.begin(), p.dependent.end(), 0);
  p.patches.resize(count);
  p.accepted.resize(p.TailOffset()+4*packet::Nodes+2*count+1);
  for (unsigned row = 0; row < count; ++row) {
    const auto& donor = original[row%original.size()];
    auto& next = p.rows[row];
    next = donor;
    next.secondary = 32+row;
    next.witnesses = {row, 1};
    p.dependent[next.secondary] = 1;
    p.present[next.secondary] = 1;
    p.fixed[packet::Nodes+next.secondary] = p.fixed[2*packet::Nodes+next.secondary] = 0;
    p.accepted[p.TailOffset()+packet::Nodes+next.secondary] = .02;
    for (unsigned axis = 0; axis < 3; ++axis)
      p.accepted[3*next.secondary+axis] = p.accepted[3*donor.secondary+axis];
    p.accepted[p.TailOffset()+4*packet::Nodes+row] = p.accepted[p.TailOffset()+next.secondary];
    p.accepted[p.TailOffset()+4*packet::Nodes+count+row] = .02;
  }
  p.accepted.back() = .125;
  p.Begin(1);
  cin_input_test::Seed(p);
  return p;
}

inline cin::StageReport FrozenForce(packet::Packet& p) {
  const auto input = p.Input();
  return frozen::PrepareForceTrial(input.model, cin_advance::force_inputs::ForceView(input));
}
inline cin::StageReport PreparedForce(packet::Packet& p, bool reverse = true) {
  auto input = p.Input();
  const auto force = cin_advance::force_inputs::ForceView(input);
  const auto check = cin::detail::CheckForceInputs(input.model, force);
  if (!check) return check;
  for (unsigned node = 0; node < packet::Nodes; ++node) force.entry_inertia[node] = force.inertia[node];
  std::vector<transfer::Row> staged(p.rows.size());
  for (unsigned index = 0; index < p.rows.size(); ++index) {
    const auto row = reverse ? p.rows.size()-1-index : index;
    staged[row].report = cin::detail::PrepareForceRow(input.model, force, row, staged[row]);
  }
  input.prepared_transfers = staged.data();
  return transfer::Apply(input);
}
inline void SamePatches(const packet::Packet& a, const packet::Packet& b) {
  ASSERT_EQ(a.patches.size(), b.patches.size());
  for (std::size_t row = 0; row < a.patches.size(); ++row) {
    EXPECT_EQ(a.patches[row].prepared(), b.patches[row].prepared());
    const auto& x = a.patches[row].values();
    const auto& y = b.patches[row].values();
    const auto same = [](tied::Vec3 x, tied::Vec3 y) {
      packet::SameDoubles({x.x,x.y,x.z}, {y.x,y.y,y.z});
    };
    same(x.center, y.center);
    same(x.secondary_offset, y.secondary_offset);
    for (unsigned i = 0; i < 4; ++i) same(x.master_offset[i], y.master_offset[i]);
    for (unsigned i = 0; i < 7; ++i) packet::SameDoubles({x.cofactor[i]}, {y.cofactor[i]});
  }
}
inline void SameForce(const packet::Packet& a, const packet::Packet& b) {
  cin_input_test::SameForce(a, b);
  SamePatches(a, b);
}
inline void LatePatch(packet::Packet& p) {
  for (unsigned slot = 0; slot < 4; ++slot) {
    p.rows.back().masters[slot] = 200+slot;
    for (unsigned axis = 0; axis < 3; ++axis) p.accepted[3*(200+slot)+axis] = 0;
  }
}
inline void CenterFirstSecondary(packet::Packet& p) {
  const auto& row = p.rows.front();
  tied::Vec3 masters[4];
  for (unsigned slot = 0; slot < 4; ++slot)
    masters[slot] = cin::detail::ReadXyz(p.accepted.data(), row.masters[slot]);
  const auto center = tied::detail::Mean(masters);
  p.accepted[3*row.secondary] = center.x;
  p.accepted[3*row.secondary+1] = center.y;
  p.accepted[3*row.secondary+2] = center.z;
}
} // namespace tl::fea::cin_transfer_test
