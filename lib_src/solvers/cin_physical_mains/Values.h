// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NodalCinPhysicalMains.h"
#include <cmath>

#if defined(__CUDACC__)
#define TL_CIN_MAIN_HD __host__ __device__
#else
#define TL_CIN_MAIN_HD
#endif
namespace tl::fea::cin_physical_mains {
inline constexpr std::size_t ValuesPerGroup = 7;
struct Values {
  tl::math::Vec3 center{};
  double mass = 0, minimum_inertia = 0, translation = 0, rotation = 0;
};
TL_CIN_MAIN_HD inline bool Valid(const Values& value) noexcept {
  return rigid::detail::Finite(value.center) && std::isfinite(value.mass) && value.mass > 0 &&
      std::isfinite(value.minimum_inertia) && value.minimum_inertia > 0 &&
      std::isfinite(value.translation) && value.translation >= 0 &&
      std::isfinite(value.rotation) && value.rotation >= 0;
}
TL_CIN_MAIN_HD inline void Write(double* output, const Values& value) noexcept {
  output[0] = value.center.x;
  output[1] = value.center.y;
  output[2] = value.center.z;
  output[3] = value.mass;
  output[4] = value.minimum_inertia;
  output[5] = value.translation;
  output[6] = value.rotation;
}
TL_CIN_MAIN_HD inline Values Read(const double* input) noexcept {
  return {{input[0],input[1],input[2]},input[3],input[4],input[5],input[6]};
}
// PhysicalAggregateV1: real initialized body M/min principal J, current center,
// native plain per-member scalar stiffness terms. PART uses this explicit
// physical policy; the unpopulated native RBYM timestep fields are not inferred.
TL_CIN_MAIN_HD inline bool Reduce(rigid::GroupDeviceView groups, std::uint32_t group,
    const double* accepted, const double* translation, const double* rotation,
    std::uint32_t nodes, Values& output, std::uint32_t& invalid_node) noexcept {
  if (!groups.groups || !groups.members || !accepted || !translation || !rotation ||
      !nodes || group >= groups.group_count) return false;
  const auto range = groups.groups[group];
  if (range.count < 2 || range.offset > groups.member_count ||
      range.count > groups.member_count-range.offset) return false;
  invalid_node = groups.members[range.offset].node;
  const auto state = rigid::ReadGroupState(accepted+19*std::size_t(nodes)+rigid::GroupStateValues*group);
  if (!rigid::ValidGroupState(state) || !std::isfinite(range.mass) || range.mass <= 0 ||
      !rigid::detail::Positive(range.principal_inertia)) return false;
  Values next;
  next.center = state.center;
  next.mass = range.mass;
  next.minimum_inertia = ::fmin(range.principal_inertia.x,
      ::fmin(range.principal_inertia.y,range.principal_inertia.z));
  for (std::uint32_t local = 0; local < range.count; ++local) {
    const auto node = groups.members[range.offset+local].node;
    invalid_node = node;
    if (node >= nodes) return false;
    const auto* position = accepted+3*node;
    const double dx = position[0]-state.center.x;
    const double dy = position[1]-state.center.y;
    const double dz = position[2]-state.center.z;
    const double kn = translation[node];
    const double kr = rotation[node];
    if (!std::isfinite(kn) || kn < 0 || !std::isfinite(kr) || kr < 0) return false;
    const double square = dx*dx+dy*dy+dz*dz;
    const double term = kr+square*kn;
    if (!std::isfinite(square) || !std::isfinite(term) || term < 0) return false;
    next.translation = next.translation+kn;
    next.rotation = next.rotation+term;
    if (!std::isfinite(next.translation) || !std::isfinite(next.rotation)) return false;
  }
  if (!Valid(next)) return false;
  output = next;
  invalid_node = UINT32_MAX;
  return true;
}
// Seven values/group fit into the dead 7*n-word CIN work tail after motion.
// Check the real preallocated capacities before any device or staging access.
inline bool Fits(std::size_t nodes, std::size_t groups, std::size_t scratch_values,
    std::size_t staging_values) noexcept {
  if (!nodes || !groups || groups > nodes/2 || nodes > SIZE_MAX/9) return false;
  const auto values = ValuesPerGroup*groups;
  return scratch_values >= 9*nodes && staging_values >= values && values <= scratch_values-2*nodes;
}
} // namespace tl::fea::cin_physical_mains
#undef TL_CIN_MAIN_HD
