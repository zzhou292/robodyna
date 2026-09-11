// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../ShellSectionLaw.h"
#include "../ShellNodalStiffness.h"
#include "../../solvers/NodalForceAssembly.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_MAPPED_GATHER_HD __host__ __device__
#else
#define TL_MAPPED_GATHER_HD
#endif

namespace tl::fea::mapped_shell {
struct AssemblyNode {
  double value[8]{}; // force XYZ, couple XYZ, translation STI, rotation STIR
  bool touched = false;
};
inline constexpr unsigned long long NoAssemblyFailure = ~0ull;

// Same numerical leaves as serial scatter, with the actual incoming values as
// the starting point. Each node retains source-parent/local-slot addition order.
template<unsigned Slots, class Memory, class ForceTrial, class BatchStatus>
TL_MAPPED_GATHER_HD inline std::uint32_t GatherNode(std::size_t node,
    const Memory& memory, const ForceTrial* accepted, const ShellSectionLaw* law,
    const DeviceNodalForceView& forces, const double* translation,
    const double* rotation, AssemblyNode& output) noexcept {
  static_assert(Slots == 3 || Slots == 4, "Only the qualified T3/QEPH slots");
  AssemblyNode next;
  const double* arrays[]{forces.force_x, forces.force_y, forces.force_z,
      forces.couple_x, forces.couple_y, forces.couple_z, translation, rotation};
  for (unsigned channel = 0; channel < 8; ++channel) next.value[channel] = arrays[channel][node];
  DeviceNodalForceView local = forces;
  local.node_count = 1;
  local.force_x = &next.value[0];
  local.force_y = &next.value[1];
  local.force_z = &next.value[2];
  local.couple_x = &next.value[3];
  local.couple_y = &next.value[4];
  local.couple_z = &next.value[5];
  const std::size_t local_node[]{0};
  for (auto index = memory.offsets[node]; index < memory.offsets[node + 1]; ++index) {
    const auto incidence = memory.incidence[index];
    const auto parent = incidence / Slots;
    const auto slot = incidence % Slots;
    if (memory.parent[parent].status != BatchStatus::Success) break;
    if (law[parent] == ShellSectionLaw::RigidSkin) continue;
    const auto& force = accepted[parent];
    const auto& stiffness = memory.parent[parent].stiffness;
    const shell_nodal_stiffness::Packet<1> packet{
        {stiffness.translation[slot]}, {stiffness.rotation[slot]}};
    if (AccumulateNodalForces<1>(local_node, force.internal_force + slot,
            force.internal_couple + slot, local, -1) != NodalForceAssemblyStatus::Success ||
        !shell_nodal_stiffness::Add(local_node, packet, &next.value[6], &next.value[7], 1)) {
      return parent;
    }
    next.touched = true;
  }
  output = next;
  return UINT32_MAX;
}

TL_MAPPED_GATHER_HD inline void PublishNode(std::size_t node, const AssemblyNode& value,
    DeviceNodalForceView forces, double* translation, double* rotation) noexcept {
  if (!value.touched) return;
  double* arrays[]{forces.force_x, forces.force_y, forces.force_z,
      forces.couple_x, forces.couple_y, forces.couple_z, translation, rotation};
  for (unsigned channel = 0; channel < 8; ++channel) arrays[channel][node] = value.value[channel];
}
} // namespace tl::fea::mapped_shell
#undef TL_MAPPED_GATHER_HD
