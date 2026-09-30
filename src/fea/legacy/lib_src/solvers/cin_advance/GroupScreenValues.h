// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Groups.h"
#include "GroupResponse.h"

namespace tl::fea::cin_advance::group_screen {
inline constexpr unsigned Threads = 64;
struct BodyValues {
  double mass, axes[9], inertia[3], center[3];
};
struct GroupState {
  BodyValues body;
  groups::Report report;
  double trace;
  bool stopped;
};
struct Tile {
  GroupState group;
  MemberResponses members[Threads];
};
static_assert(std::is_trivial<Tile>::value);
static_assert(sizeof(BodyValues) == 128);
static_assert(sizeof(GroupState) == 168);
static_assert(sizeof(Tile) == 4776);

TL_SURFACE_HD inline BodyValues StoreBody(const tlfea::contact::RigidContactBody& body) noexcept {
  BodyValues out{};
  out.mass = body.mass;
  for (unsigned i = 0; i < 9; ++i) out.axes[i] = body.current_frame.axes.v[i];
  out.inertia[0] = body.current_frame.inertia.x;
  out.inertia[1] = body.current_frame.inertia.y;
  out.inertia[2] = body.current_frame.inertia.z;
  out.center[0] = body.center.x;
  out.center[1] = body.center.y;
  out.center[2] = body.center.z;
  return out;
}
TL_SURFACE_HD inline tlfea::contact::RigidContactBody ReadBody(const BodyValues& values) noexcept {
  tlfea::contact::RigidContactBody out;
  out.mass = values.mass;
  for (unsigned i = 0; i < 9; ++i) out.current_frame.axes.v[i] = values.axes[i];
  out.current_frame.inertia = {values.inertia[0], values.inertia[1], values.inertia[2]};
  out.center = {values.center[0], values.center[1], values.center[2]};
  return out;
}
TL_SURFACE_HD inline GroupState BeginGroup(const cin_timestep::Sources& source,
    std::uint32_t group) noexcept {
  GroupState out{};
  out.stopped = true;
  out.report = {std::numeric_limits<double>::max(), UINT32_MAX, UINT32_MAX,
      NodalStatus::InvalidOutput, false, false};
  const auto groups = source.rigid;
  const auto range = groups.groups[group];
  if (range.count < 2 || range.offset > groups.member_count ||
      range.count > groups.member_count-range.offset) return out;
  out.report.visited = true;
  out.report.first_node = out.report.last_node = groups.members[range.offset].node;
  const auto prior = rigid::ReadGroupState(source.accepted+19*std::size_t(source.nodes)+
      rigid::GroupStateValues*group);
  tlfea::contact::RigidContactBody body;
  if (tlfea::contact::PrepareRigidContactBodyFromAccepted(prior, range.mass,
      range.principal_inertia, source.previous_drift_dt, body) != tlfea::contact::Status::kOk)
    return out;
  out.body = StoreBody(body);
  out.stopped = false;
  return out;
}
TL_SURFACE_HD inline MemberResponses PrepareMember(const cin_timestep::Sources& source,
    const tlfea::contact::RigidContactBody& body, std::uint32_t member) noexcept {
  const auto node = source.rigid.members[member].node;
  if (node >= source.nodes || source.cin_secondary[node] || !source.rigid.member_nodes[node])
    return {};
  const auto* x = source.accepted+3*node;
  return PrepareResponses(body, {x[0], x[1], x[2]}, source.translation[node], source.rotation[node]);
}
TL_SURFACE_HD inline void FoldMember(const cin_timestep::Sources& source,
    const tlfea::contact::RigidContactBody& body, std::uint32_t member,
    const MemberResponses& responses, GroupState& state) noexcept {
  if (state.stopped) return;
  const auto node = source.rigid.members[member].node;
  state.report.last_node = node;
  if (node >= source.nodes || source.cin_secondary[node] || !source.rigid.member_nodes[node]) {
    state.stopped = true;
    return;
  }
  const auto* x = source.accepted+3*node;
  if (!cin_timestep::detail::AddRigidMemberTraceWithResponse(body, {x[0], x[1], x[2]},
      source.translation[node], source.rotation[node], state.trace, ReadPreparedResponse{responses}))
    state.stopped = true;
}
TL_SURFACE_HD inline void FinishGroup(double factor, GroupState& state) noexcept {
  if (state.stopped) return;
  cin_timestep::ScalarLimit limit;
  if (!cin_timestep::RigidTraceLimit(state.trace, factor, limit)) {
    state.stopped = true;
    return;
  }
  state.report.minimum_dt = limit.dt;
  state.report.bounded = limit.bounded;
  state.report.status = NodalStatus::Ok;
}
} // namespace tl::fea::cin_advance::group_screen
