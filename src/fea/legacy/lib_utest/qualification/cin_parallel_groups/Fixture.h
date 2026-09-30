// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../cin_parallel_ordinary/Packet.h"
#include "lib_src/solvers/cin_advance/Groups.h"
#include "lib_src/solvers/cin_advance/Screen.h"
#include "lib_src/solvers/cin_timestep/Screen.h"
#include <algorithm>

namespace tl::fea::cin_group_test {
namespace packet = cin_parallel_test;
// Reuse the admitted CIN packet. Only the independent group population changes;
// all two-node bodies have distinct endpoints outside the shared-master patch.
inline void PopulateGroups(packet::Packet& p, unsigned count) {
  const auto old_tail = p.TailOffset();
  const std::vector<double> tail(p.accepted.begin()+old_tail, p.accepted.end());
  p.groups.assign(count, {});
  p.members.assign(2*count, {});
  p.member_nodes.assign(packet::Nodes, 0);
  p.accepted.resize(p.TailOffset()+tail.size());
  std::copy(tail.begin(), tail.end(), p.accepted.begin()+p.TailOffset());
  p.capture.assign(6*(packet::Nodes+count), -91);
  for (unsigned g = 0; g < count; ++g) {
    p.groups[g] = {2*g, 2, 2, {.2, 2.2, 2.2}, true};
    NodalRigidGroupState state;
    state.center = {20+double(g), .5, -.5};
    state.velocity = {.01, -.02, .03};
    state.omega = {.001, .002, -.003};
    state.principal_axes = {{1,0,0,0,1,0,0,0,1}};
    rigid::WriteGroupState(p.accepted.data()+19*packet::Nodes+rigid::GroupStateValues*g, state);
    for (unsigned k = 0; k < 2; ++k) {
      const auto n = packet::Nodes-2*count+2*g+k;
      p.members[2*g+k] = {n, 1, .1};
      p.member_nodes[n] = g%2 ? rigid::PhysicalPlainMemberNode : rigid::PartMemberNode;
      p.fixed[packet::Nodes+n] = p.fixed[2*packet::Nodes+n] = 0;
      p.present[n] = 1;
      p.accepted[p.TailOffset()+n] = 1;
      p.accepted[p.TailOffset()+packet::Nodes+n] = .1;
      p.accepted[3*n] = state.center.x+2*k-1;
      p.accepted[3*n+1] = state.center.y;
      p.accepted[3*n+2] = state.center.z;
    }
  }
  p.Begin(p.attempt);
}
inline void ReverseGroups(packet::Packet& p) {
  std::reverse(p.groups.begin(), p.groups.end());
  for (std::size_t g = 0; g < p.groups.size()/2; ++g)
    for (unsigned a = 0; a < rigid::GroupStateValues; ++a)
      std::swap(p.accepted[19*packet::Nodes+rigid::GroupStateValues*g+a],
        p.accepted[19*packet::Nodes+rigid::GroupStateValues*(p.groups.size()-1-g)+a]);
  p.Begin(p.attempt);
}
inline packet::Packet MakePacket(unsigned count, bool capture) {
  // Only the original two-body case starts with physical zero-M/J members.
  // Building a zero-group case from it would leave those nodes ordinary/free.
  packet::Packet result(count == 2, capture);
  if (count != 2) PopulateGroups(result, count);
  ReverseGroups(result);
  return result;
}
} // namespace tl::fea::cin_group_test
