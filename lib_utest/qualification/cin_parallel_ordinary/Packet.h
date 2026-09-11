// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Serial.h"
#include <vector>

namespace tl::fea::cin_parallel_test {
namespace cin = constraints::tied_shell::cin;
namespace tied = constraints::tied_shell;
inline constexpr std::uint32_t Nodes = 272;
inline constexpr double H = 1e-6;
struct Packet {
  std::vector<cin::StageRow> rows;
  std::vector<rigid::GroupRange> groups;
  std::vector<rigid::MemberMetric> members;
  std::vector<std::uint8_t> dependent, activity, member_nodes, fixed, present;
  std::vector<double> accepted, trial, loads, work, capture, stiffness;
  std::vector<tied::Patch> patches;
  nodal_detail::Control control;
  cin_advance::FailureKey failure = 7;
  rigid::StepDurations durations{0, H/2, H};
  NodalCinStructuralStep structural;
  std::uint64_t epoch = 0, attempt = 1;
  bool capture_enabled = false;
  explicit Packet(bool with_groups = false, bool with_capture = false);
  std::size_t TailOffset() const { return 19*Nodes+rigid::GroupStateValues*groups.size(); }
  cin_advance::Input Input();
  void Begin(std::uint64_t next_attempt);
  void Accept();
};
void SameControl(const nodal_detail::Control&, const nodal_detail::Control&);
void SameSuccessfulPacket(const Packet&, const Packet&);
void SameDoubles(const std::vector<double>&, const std::vector<double>&);
} // namespace tl::fea::cin_parallel_test
