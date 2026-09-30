// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_force_transfers/Fixture.h"
#include "lib_src/solvers/cin_advance/Recovery.h"
#include "lib_src/solvers/cin_advance/RecoveryDrift.h"
#include "lib_src/constraints/tied_shell/runtime/CinMotionStage.h"
#include "FrozenMotion.h"

namespace tl::fea::cin_recovery_test {
namespace packet = cin_parallel_test;
namespace cin = constraints::tied_shell::cin;
namespace tied = constraints::tied_shell;
namespace recovery = cin_advance::recovery;
using cin_transfer_test::Population;
inline void BeginInterval(packet::Packet& state, std::uint64_t attempt,
    const std::vector<double>& assembly_loads) {
  state.Begin(attempt);
  cin_input_test::Seed(state);
  state.loads = assembly_loads;
}
inline packet::Packet MotionPacket(unsigned count, bool skewed_geometry = false) {
  auto result = Population(count, true, true);
  if (skewed_geometry) {
    for (unsigned node = 0; node < packet::Nodes; ++node) {
      const auto x = cin::detail::ReadXyz(result.accepted.data(), node);
      result.accepted[3*node] = .7*x.x+.2*x.y-.1*x.z+3;
      result.accepted[3*node+1] = 1.2*x.y+.1*x.z+.3;
      result.accepted[3*node+2] = .2*x.x+1.4*x.z-2;
    }
  }
  if (!cin_transfer_test::PreparedForce(result)) throw std::runtime_error("Motion packet force preparation failed");
  result.failure = cin_advance::NoFailure;
  for (unsigned node = 0; node < packet::Nodes; ++node) {
    for (unsigned axis = 0; axis < 3; ++axis) {
      const auto i = 3*node+axis;
      result.work[3*packet::Nodes+i] = .003*(node%9+axis);
      result.work[6*packet::Nodes+i] = -.001*(node%7+axis);
      if (result.dependent[node]) {
        result.trial[3*packet::Nodes+i] = -11-node;
        result.trial[6*packet::Nodes+i] = -12-node;
      }
    }
  }
  return result;
}
inline cin::StageReport Serial(packet::Packet& packet) {
  const auto input = packet.Input();
  return tied::cin_recovery_frozen::RecoverMotionTrial(input.model, recovery::MotionView(input));
}
inline cin::StageReport Prepared(packet::Packet& packet, std::vector<recovery::Row>& rows) {
  auto input = packet.Input();
  if (!cin::detail::MotionPointersValid(input.model, recovery::MotionView(input)))
    return {cin::StageStatus::InvalidInput};
  input.prepared_recovery = rows.data();
  auto first = recovery::NoFailure;
  for (unsigned i = 0; i < rows.size(); ++i) {
    const auto row = unsigned(rows.size()-1-i);
    rows[row] = recovery::Prepare(input, row);
    if (!rows[row].valid) first = std::min(first, row);
  }
  for (unsigned i = 0; i < rows.size(); ++i)
    recovery::Publish(input, unsigned(rows.size()-1-i), first);
  return first == recovery::NoFailure ? cin::StageReport{}
      : cin::StageReport{cin::StageStatus::NonfiniteResult, first, input.model.rows[first].secondary};
}
inline void Same(const packet::Packet& a, const packet::Packet& b) {
  packet::SameControl(a.control, b.control);
  cin_transfer_test::SameForce(a, b);
  packet::SameDoubles(a.accepted, b.accepted);
  packet::SameDoubles(a.capture, b.capture);
}
cudaError_t LaunchFrozen(const cin_advance::Input&, cudaStream_t);
} // namespace tl::fea::cin_recovery_test
