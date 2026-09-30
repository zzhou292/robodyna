// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Storage.h"
#if defined(__CUDACC__)
#define TL_CONNECTOR_HD __host__ __device__
#else
#define TL_CONNECTOR_HD
#endif
namespace tl::fea::mapped_connector {
TL_CONNECTOR_HD inline unsigned long long ParentKey(Order order, std::size_t parents,
    std::size_t parent, Failure failure) noexcept {
  const auto phase = order == Order::AllEndpointsFirst && failure != Failure::Endpoint ? 2ull * parents : 0;
  return phase + 2ull * parent;
}
TL_CONNECTOR_HD inline unsigned long long AdditionKey(Order order, std::size_t parents,
    std::size_t parent) noexcept {
  return ParentKey(order, parents, parent, Failure::Assembly) + 1;
}
TL_CONNECTOR_HD inline std::size_t FailedParent(Order order, std::size_t parents,
    unsigned long long key) noexcept {
  if (order == Order::AllEndpointsFirst && key >= 2ull * parents) key -= 2ull * parents;
  return static_cast<std::size_t>(key / 2);
}
TL_CONNECTOR_HD inline bool ForceViewValid(const DeviceNodalForceView& view,
    const std::size_t (&nodes)[2]) noexcept {
  const double* arrays[]{view.force_x, view.force_y, view.force_z,
      view.couple_x, view.couple_y, view.couple_z};
  if (!view.node_count || nodes[0] == nodes[1] || nodes[0] >= view.node_count || nodes[1] >= view.node_count)
    return false;
  for (unsigned i = 0; i < 6; ++i) {
    if (!arrays[i]) return false;
    for (unsigned j = 0; j < i; ++j) if (arrays[i] == arrays[j]) return false;
  }
  return true;
}
TL_CONNECTOR_HD inline bool StiffnessViewValid(const double* translation, const double* rotation,
    std::size_t count, const std::size_t (&nodes)[2]) noexcept {
  return translation && rotation && translation != rotation && nodes[0] != nodes[1] &&
      nodes[0] < count && nodes[1] < count;
}
TL_CONNECTOR_HD inline bool StiffnessSum(double translation, double rotation,
    const mapped_shell::AssemblyNode& before, double& next_translation, double& next_rotation) noexcept {
  next_translation = before.value[6] + translation;
  next_rotation = before.value[7] + rotation;
  return tl::math::Finite(before.value[6]) && before.value[6] >= 0 &&
      tl::math::Finite(before.value[7]) && before.value[7] >= 0 &&
      tl::math::Finite(next_translation) && next_translation >= 0 &&
      tl::math::Finite(next_rotation) && next_rotation >= 0;
}
// Same force leaf and additions as the two-endpoint scatter. Only independent
// nodes run concurrently; incidence order within a node is never reduced/reordered.
template<bool StiffnessFirst, class Evaluation>
TL_CONNECTOR_HD inline bool AddEndpoint(const Parent& parent, const Evaluation& value,
    unsigned slot, mapped_shell::AssemblyNode& next) noexcept {
  DeviceNodalForceView local{};
  local.node_count = 1;
  local.force_x = &next.value[0]; local.force_y = &next.value[1]; local.force_z = &next.value[2];
  local.couple_x = &next.value[3]; local.couple_y = &next.value[4]; local.couple_z = &next.value[5];
  double translation = 0, rotation = 0;
  if (StiffnessFirst && !StiffnessSum(parent.translation, parent.rotation, next, translation, rotation)) return false;
  const std::size_t node[]{0};
  if (AccumulateNodalForces<1>(node, &value.endpoints[slot].force_N,
          &value.endpoints[slot].couple_Nm, local, +1) != NodalForceAssemblyStatus::Success) return false;
  if (!StiffnessFirst && !StiffnessSum(parent.translation, parent.rotation, next, translation, rotation)) return false;
  next.value[6] = translation; next.value[7] = rotation;
  return true;
}
template<class Access>
TL_CONNECTOR_HD inline std::uint32_t GatherNode(std::size_t node, const Memory& memory,
    const Access& values, DeviceNodalForceView forces, const double* translation,
    const double* rotation, mapped_shell::AssemblyNode& output) noexcept {
  mapped_shell::AssemblyNode next;
  for (auto i = memory.offsets[node]; i < memory.offsets[node + 1]; ++i) {
    const auto parent = memory.incidence[i] / 2;
    const auto slot = memory.incidence[i] % 2;
    const auto& record = memory.parent[parent];
    if (record.failure != Failure::None) break;
    if (!next.touched) {
      const double* arrays[]{forces.force_x, forces.force_y, forces.force_z,
          forces.couple_x, forces.couple_y, forces.couple_z, translation, rotation};
      for (unsigned channel = 0; channel < 8; ++channel) next.value[channel] = arrays[channel][node];
    }
    if (!values.Add(parent, slot, record, next)) return parent;
    next.touched = true;
  }
  output = next;
  return UINT32_MAX;
}
} // namespace tl::fea::mapped_connector
#undef TL_CONNECTOR_HD
