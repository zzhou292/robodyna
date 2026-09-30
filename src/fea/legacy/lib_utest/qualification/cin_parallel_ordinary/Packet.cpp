// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Packet.h"
#include "lib_utest/qualification/tied_cin_runtime/Fixture.h"
#include <gtest/gtest.h>
#include <algorithm>

namespace tl::fea::cin_parallel_test {
Packet::Packet(bool with_groups, bool with_capture) : capture_enabled(with_capture) {
  // Reuse the admitted attachment fixture, including its repeated T3 slot.
  cin_runtime_test::Fixture source;
  rows = source.rows;
  dependent.resize(Nodes);
  activity = source.flags;
  member_nodes.resize(Nodes);
  fixed.resize(3*Nodes);
  present.assign(Nodes, 1);
  if (with_groups) {
    groups.resize(2);
    members.resize(6);
  }
  const auto state_size = TailOffset()+4*Nodes+2*rows.size()+1;
  accepted.resize(state_size);
  loads.resize(6*Nodes);
  work.resize(9*Nodes);
  stiffness.resize(2*Nodes);
  capture.resize(6*(Nodes+groups.size()), -91);
  patches.resize(rows.size());
  const auto source_nodes = source.mass.size();
  for (std::uint32_t i=0; i<Nodes; ++i) {
    accepted[9*Nodes+4*i] = 1;
    for (unsigned a=0; a<3; ++a) {
      accepted[3*i+a] = i<source_nodes ? source.x[3*i+a] : .001*i+.1*a;
      accepted[3*Nodes+3*i+a] = .002*(i%7+a);
      accepted[6*Nodes+3*i+a] = .0003*(i%5+a);
      loads[a*Nodes+i] = .125*(1+i%5+a);
      loads[(a+3)*Nodes+i] = .001*(1+i%3+a);
    }
    accepted[TailOffset()+i] = 2+.125*(i%9);
    accepted[TailOffset()+Nodes+i] = .01+.001*(i%7);
    stiffness[i] = 4+i%11;
    stiffness[Nodes+i] = .1+.01*(i%5);
  }
  for (std::size_t r=0; r<rows.size(); ++r) {
    const auto i = rows[r].secondary;
    dependent[i] = 1;
    // Exact INIEND initial SMAS/SINER seed, separate from current transferred M/J.
    accepted[TailOffset()+4*Nodes+r] = accepted[TailOffset()+i];
    accepted[TailOffset()+4*Nodes+rows.size()+r] = accepted[TailOffset()+Nodes+i];
  }
  // Disjoint ordinary fixed/absent cases beyond the original attachment nodes.
  fixed[Nodes+130] = 5;
  fixed[2*Nodes+131] = 1;
  accepted[TailOffset()+Nodes+131] = 0;
  present[132] = 0;
  accepted[TailOffset()+Nodes+132] = 0;
  stiffness[Nodes+132] = 0;
  for (unsigned a=0; a<3; ++a) {
    accepted[6*Nodes+3*132+a] = 0;
    loads[(a+3)*Nodes+132] = 0;
  }
  for (std::size_t g=0; g<groups.size(); ++g) {
    // Selected caller values: two disjoint three-member bodies with a zero
    // physical M/J dependent. Their source/aggregate admission is independently
    // exercised by rigid_assembly_owner; this packet tests stage scheduling.
    const auto first = Nodes-6+3*g;
    groups[g] = {std::uint32_t(3*g), 3, 2, {.2, 2.2, 2.2}, true};
    NodalRigidGroupState state;
    state.center = {20+double(g), 0, 0};
    state.velocity = {.01, -.02, .03};
    state.omega = {.001, .002, -.003};
    state.principal_axes = {{1,0,0,0,1,0,0,0,1}};
    rigid::WriteGroupState(accepted.data()+19*Nodes+rigid::GroupStateValues*g, state);
    for (unsigned k=0; k<3; ++k) {
      const auto i = first+k;
      const double m = k==1 ? 0 : 1;
      const double j = k==1 ? 0 : .1;
      members[3*g+k] = {std::uint32_t(i), m, j};
      member_nodes[i] = g==0 ? rigid::PartMemberNode : rigid::PhysicalPlainMemberNode;
      accepted[TailOffset()+i] = m;
      accepted[TailOffset()+Nodes+i] = j;
      accepted[3*i] = state.center.x+double(k)-1;
      accepted[3*i+1] = 0;
      accepted[3*i+2] = 0;
    }
  }
  Begin(1);
}

void Packet::Begin(std::uint64_t next_attempt) {
  attempt = next_attempt;
  trial = accepted;
  std::fill(work.begin(), work.end(), 0);
  std::copy(stiffness.begin(), stiffness.end(), work.begin());
  std::fill(capture.begin(), capture.end(), -91);
  control = {};
  control.rows = {nullptr, nullptr, Nodes, Nodes, epoch, attempt, true, true, true};
  control.limit.dt = H;
  control.limit.base_epoch = epoch;
  control.limit.attempt = attempt;
  // Deliberately stale. The production prefix must reset this every attempt.
  failure = cin_advance::EncodeFailure(0, NodalStatus::InvalidOutput);
}

void Packet::Accept() {
  accepted = trial;
  ++epoch;
  durations = {H, H, H};
}

cin_advance::Input Packet::Input() {
  const rigid::GroupDeviceView group_view{groups.data(), members.data(),
    groups.empty()?nullptr:member_nodes.data(), std::uint32_t(groups.size()),
    std::uint32_t(members.size()), .001};
  rigid::AccelerationSink sink;
  if (capture_enabled) sink = {capture.data(), capture.data()+3*Nodes,
    capture.data()+6*Nodes, capture.data()+6*Nodes+3*groups.size()};
  return {&control, accepted.data(), trial.data(), loads.data(), fixed.data(),
    {rows.data(), dependent.data(), Nodes, std::uint32_t(rows.size()), std::uint32_t(activity.size()),
      first_witness.empty()?nullptr:first_witness.data()},
    trial.data()+TailOffset(), work.data(), patches.data(), activity.data(), group_view,
    durations, .2, epoch, attempt, sink, present.data(), structural, &failure};
}
} // namespace tl::fea::cin_parallel_test
