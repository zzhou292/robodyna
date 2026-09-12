// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Sources.h"

namespace tl::fea::cin_timestep::detail {
TL_SURFACE_HD inline bool CheckSources(const Sources& source, double factor) noexcept {
  if (!source.accepted || !source.mass || !source.inertia || !source.translation ||
      !source.rotation || !source.fixed_translation || !source.fixed_rotation ||
      !source.cin_secondary || !source.nodes || !std::isfinite(source.previous_drift_dt) ||
      source.previous_drift_dt < 0 || !std::isfinite(factor) || factor <= 0 || factor > 1)
    return false;
  const auto groups = source.rigid;
  if (groups.group_count && (!groups.groups || !groups.members || !groups.member_nodes)) return false;
  return true;
}
TL_SURFACE_HD inline bool EvaluateNode(const Sources& source, double factor,
    std::uint32_t node, Result& next) noexcept {
  const auto groups = source.rigid;
  const double coefficients[]{source.mass[node], source.inertia[node],
    source.translation[node], source.rotation[node]};
  for (const double value : coefficients) {
    if (!std::isfinite(value) || value < 0) return false;
  }
  if (source.cin_secondary[node]) return true;
  if (groups.member_nodes && groups.member_nodes[node]) return true;
  ScalarLimit limit;
  if (source.fixed_translation[node] != 7) {
    if (!OrdinaryLimit(source.mass[node], source.translation[node], factor, limit)) return false;
    Include(limit, node, UINT32_MAX, next);
  }
  if (source.rotation_present && !source.rotation_present[node]) {
    if (source.rotation[node] != 0) return false;
  } else if (!source.fixed_rotation[node]) {
    if (!OrdinaryLimit(source.inertia[node], source.rotation[node], factor, limit)) return false;
    Include(limit, node, UINT32_MAX, next);
  }
  return true;
}
TL_SURFACE_HD inline bool EvaluateGroups(const Sources& source, double factor,
    Result& next, std::uint32_t& invalid_node) noexcept {
  const auto groups = source.rigid;
  for (std::uint32_t group = 0; group < groups.group_count; ++group) {
    const auto range = groups.groups[group];
    if (range.count < 2 || range.offset > groups.member_count ||
        range.count > groups.member_count-range.offset) return false;
    invalid_node = groups.members[range.offset].node;
    const auto prior = rigid::ReadGroupState(source.accepted+19*std::size_t(source.nodes)+
        rigid::GroupStateValues*group);
    tlfea::contact::RigidContactBody body;
    if (tlfea::contact::PrepareRigidContactBodyFromAccepted(prior, range.mass,
        range.principal_inertia, source.previous_drift_dt, body) != tlfea::contact::Status::kOk)
      return false;
    double trace = 0;
    for (std::uint32_t local = 0; local < range.count; ++local) {
      const auto node = groups.members[range.offset+local].node;
      invalid_node = node;
      if (node >= source.nodes || source.cin_secondary[node] || !groups.member_nodes[node]) return false;
      const auto* x = source.accepted+3*node;
      if (!AddRigidMemberTrace(body, {x[0], x[1], x[2]}, source.translation[node],
          source.rotation[node], trace)) return false;
    }
    ScalarLimit limit;
    if (!RigidTraceLimit(trace, factor, limit)) return false;
    Include(limit, groups.members[range.offset].node, group, next);
  }
  return true;
}
} // namespace tl::fea::cin_timestep::detail
