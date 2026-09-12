// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/cin_timestep/Sources.h"
#include "FrozenRigid.h"

namespace tl::fea::cooperative_test::frozen {
using namespace cin_timestep;
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
struct GroupEvaluation {
  ScalarLimit limit;
  std::uint32_t first_node = UINT32_MAX, last_node = UINT32_MAX;
  bool visited = false, valid = false;
};
// The exact original group body; a malformed range consumes no member. The
// caller retains the preceding invalid-node value in that branch.
TL_SURFACE_HD inline GroupEvaluation EvaluateGroup(const Sources& source, double factor,
    std::uint32_t group, double* trace_upper = nullptr) noexcept {
  GroupEvaluation out;
  const auto groups = source.rigid;
  const auto range = groups.groups[group];
  if (range.count < 2 || range.offset > groups.member_count ||
      range.count > groups.member_count-range.offset) return out;
  out.visited = true;
  out.first_node = out.last_node = groups.members[range.offset].node;
  const auto prior = rigid::ReadGroupState(source.accepted+19*std::size_t(source.nodes)+
      rigid::GroupStateValues*group);
  tlfea::contact::RigidContactBody body;
  if (tlfea::contact::PrepareRigidContactBodyFromAccepted(prior, range.mass,
      range.principal_inertia, source.previous_drift_dt, body) != tlfea::contact::Status::kOk)
    return out;
  double trace = 0;
  for (std::uint32_t local = 0; local < range.count; ++local) {
    const auto node = groups.members[range.offset+local].node;
    out.last_node = node;
    if (node >= source.nodes || source.cin_secondary[node] || !groups.member_nodes[node]) return out;
    const auto* x = source.accepted+3*node;
    if (!frozen::AddRigidMemberTrace(body, {x[0], x[1], x[2]}, source.translation[node],
        source.rotation[node], trace)) return out;
  }
  if (!frozen::RigidTraceLimit(trace, factor, out.limit)) return out;
  if (trace_upper) *trace_upper = trace;
  out.valid = true;
  return out;
}
TL_SURFACE_HD inline bool EvaluateGroups(const Sources& source, double factor,
    Result& next, std::uint32_t& invalid_node) noexcept {
  for (std::uint32_t group = 0; group < source.rigid.group_count; ++group) {
    const auto value = EvaluateGroup(source, factor, group);
    if (value.visited) invalid_node = value.last_node;
    if (!value.valid) return false;
    Include(value.limit, value.first_node, group, next);
  }
  return true;
}
} // namespace tl::fea::cin_timestep::detail
