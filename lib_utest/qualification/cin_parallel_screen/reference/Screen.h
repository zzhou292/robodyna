// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Rigid.h"

namespace tl::fea::cin_timestep {
struct Sources {
  const double* accepted = nullptr; // Existing x3/v3/w3/q4/reaction6 + group tail.
  const double* mass = nullptr;
  const double* inertia = nullptr;
  const double* translation = nullptr;
  const double* rotation = nullptr;
  const std::uint8_t* fixed_translation = nullptr;
  const std::uint8_t* fixed_rotation = nullptr;
  const std::uint8_t* rotation_present = nullptr; // Null means all present.
  const std::uint8_t* cin_secondary = nullptr;
  rigid::GroupDeviceView rigid;
  std::uint32_t nodes = 0;
  double previous_drift_dt = 0;
};
struct Result {
  double minimum_dt = std::numeric_limits<double>::max();
  std::uint32_t limiting_node = UINT32_MAX;
  std::uint32_t limiting_group = UINT32_MAX;
  bool valid = false;
};
TL_SURFACE_HD inline void Include(const ScalarLimit& limit, std::uint32_t node,
    std::uint32_t group, Result& result) noexcept {
  if (limit.bounded && limit.dt < result.minimum_dt) {
    result.minimum_dt = limit.dt;
    result.limiting_node = node;
    result.limiting_group = group;
  }
}
// The sole owner supplies its startup-validated complete membership and role
// maps; this value view does not independently authenticate or construct them.
// Pure complete source traversal. On failure only invalid_node is returned via
// the separate error parameter; output remains unchanged. No M/J or motion write.
TL_SURFACE_HD inline bool Screen(const Sources& source, double factor, Result& output,
    std::uint32_t& invalid_node) noexcept {
  if (!source.accepted || !source.mass || !source.inertia || !source.translation ||
      !source.rotation || !source.fixed_translation || !source.fixed_rotation ||
      !source.cin_secondary || !source.nodes || !std::isfinite(source.previous_drift_dt) ||
      source.previous_drift_dt < 0 || !std::isfinite(factor) || factor <= 0 || factor > 1)
    return false;
  const auto groups = source.rigid;
  if (groups.group_count && (!groups.groups || !groups.members || !groups.member_nodes)) return false;
  Result next;
  for (std::uint32_t node = 0; node < source.nodes; ++node) {
    invalid_node = node;
    const double coefficients[]{source.mass[node], source.inertia[node],
      source.translation[node], source.rotation[node]};
    for (const double value : coefficients) {
      if (!std::isfinite(value) || value < 0) return false;
    }
    if (source.cin_secondary[node]) continue;
    if (groups.member_nodes && groups.member_nodes[node]) continue;
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
  }
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
  next.valid = true;
  invalid_node = UINT32_MAX;
  output = next;
  return true;
}
} // namespace tl::fea::cin_timestep
