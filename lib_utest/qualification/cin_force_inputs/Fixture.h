// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_parallel_ordinary/Packet.h"
#include "serial/Force.h"
#include "lib_src/solvers/cin_advance/ForceInputs.h"
#include <gtest/gtest.h>
#include <cmath>
#include <algorithm>
#include <numeric>

namespace tl::fea::cin_input_test {
using namespace cin_parallel_test;
namespace inputs = cin_advance::force_inputs;
namespace frozen = constraints::tied_shell::cin_input_frozen;

inline void Seed(Packet& p) {
  std::fill(p.work.begin()+2*Nodes, p.work.begin()+3*Nodes, -9876.25);
  p.first_witness.resize(p.activity.size());
  std::iota(p.first_witness.begin(), p.first_witness.end(), 0u);
  p.input_failure = 19;
}
inline cin::StageReport StagedForce(Packet& p, bool reverse = false) {
  auto input = p.Input();
  const auto force = inputs::ForceView(input);
  auto result = cin::detail::CheckForcePointers(input.model, force);
  if (!result) return result;
  auto first = cin_advance::NoFailure;
  for (std::uint32_t index = 0; index < Nodes; ++index) {
    const auto node = reverse ? Nodes-1-index : index;
    const auto report = cin::detail::CheckForceNode(input.model, force, node);
    if (!report) first = std::min(first, static_cast<cin_advance::FailureKey>(node));
  }
  if (first != cin_advance::NoFailure) {
    return {cin::StageStatus::InvalidInput, UINT32_MAX, std::uint32_t(first)};
  }
  result = cin::detail::CheckForceAfterNodes(input.model, force);
  if (!result) return result;
  for (std::uint32_t index = 0; index < Nodes; ++index) {
    const auto node = reverse ? Nodes-1-index : index;
    force.entry_inertia[node] = force.inertia[node];
  }
  return cin::detail::TransferForceTrial(input.model, force);
}
inline cin::StageReport FrozenForce(Packet& p) {
  const auto input = p.Input();
  return frozen::PrepareForceTrial(input.model, inputs::ForceView(input));
}
inline void SameReport(cin::StageReport a, cin::StageReport b) {
  EXPECT_EQ(a.status, b.status);
  EXPECT_EQ(a.row, b.row);
  EXPECT_EQ(a.node, b.node);
}
inline void SameForce(const Packet& a, const Packet& b) {
  SameDoubles(a.accepted, b.accepted);
  SameDoubles(a.trial, b.trial);
  SameDoubles(a.loads, b.loads);
  SameDoubles(a.work, b.work);
  SameDoubles(a.capture, b.capture);
}
// Every listed fault rejects before the entry-IN copy; later transfer/motion
// failures retain the existing qualification fixture and native controls.
enum class Fault {
  EarlyMass, BoundaryInertia, LastPosition, LastCouple, LateStiffness,
  NodeBeforeWitness, NodeBeforeNumerical, NumericalBeforeWitness,
  WitnessBeforeRow, WitnessAlias, LateRow, MissingActivity, NoRows,
  StaleBeforeNode, LimitBeforeNode
};
inline void Inject(Packet& p, Fault fault) {
  const auto tail = p.TailOffset();
  const auto numerical = tail+4*Nodes+2*p.rows.size();
  switch (fault) {
    case Fault::EarlyMass: p.trial[tail] = -1; break;
    case Fault::BoundaryInertia:
      p.trial[tail+Nodes+128] = NAN;
      p.trial[tail+Nodes+127] = -1;
      break;
    case Fault::LastPosition: p.accepted[3*(Nodes-1)+2] = INFINITY; break;
    case Fault::LastCouple: p.loads[5*Nodes+Nodes-1] = NAN; break;
    case Fault::LateStiffness: p.work[Nodes+256] = -1; break;
    case Fault::NodeBeforeWitness:
      p.work[128] = NAN;
      p.activity.back() = 0;
      break;
    case Fault::NodeBeforeNumerical:
      p.work[128] = NAN;
      p.trial[numerical] = INFINITY;
      break;
    case Fault::NumericalBeforeWitness:
      p.trial[numerical] = NAN;
      p.activity.back() = 0;
      break;
    case Fault::WitnessBeforeRow:
      p.first_witness[0] = 1;
      p.rows.back().masters[0] = Nodes;
      break;
    case Fault::WitnessAlias:
      p.first_witness.back() = 0;
      p.activity.back() = 2;
      break;
    case Fault::LateRow: p.rows.back().masters[3] = Nodes; break;
    case Fault::MissingActivity:
      p.activity.clear();
      p.work[128] = NAN;
      break;
    case Fault::NoRows:
      p.rows.clear();
      p.work[128] = NAN;
      break;
    case Fault::StaleBeforeNode:
      ++p.control.rows.attempt;
      p.work[128] = NAN;
      break;
    case Fault::LimitBeforeNode:
      p.control.limit.has_stiffness_or_damping = true;
      p.work[128] = NAN;
      break;
  }
}
inline void UnchangedBeforeTransfer(const Packet& after, const Packet& before) {
  SameForce(after, before);
  EXPECT_EQ(after.failure, before.failure);
  for (std::size_t row = 0; row < before.patches.size(); ++row) {
    EXPECT_EQ(after.patches[row].prepared(), before.patches[row].prepared());
  }
}
} // namespace tl::fea::cin_input_test
