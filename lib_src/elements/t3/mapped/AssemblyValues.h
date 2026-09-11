// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "AssemblyTypes.h"
#include "Result.h"
#include "../../ShellMappedNode.h"

namespace tl::fea::t3::mapped {
template<class Model>
TL_T3_HD inline AssemblyParent PrepareAssemblyParent(const Model& model, const ForceTrial& result,
    std::size_t parent, ShellSectionLaw law, const NodalAssemblyView& view, bool initial) noexcept {
  AssemblyParent next;
  const auto& element = model.element[parent];
  for (auto node : element.nodes) {
    if (shell_mapped_detail::ValidNode(model, view, node, initial)) continue;
    next.status = BatchStatus::InvalidInput;
    next.node = static_cast<std::uint32_t>(node);
    return next;
  }
  const bool skin = law == ShellSectionLaw::RigidSkin;
  if (!ValidResult(element.reference, result, view.position_time, view.accepted.base_epoch, skin) ||
      (!skin && !(initial ? InitialStiffness(element.reference, law, next.stiffness) :
          AcceptedStiffness(result, next.stiffness)))) {
    next.status = BatchStatus::NonfiniteResult;
  } else if (!skin) {
    // The serial Add<3> rejects duplicate local slots. Immutable incidence is
    // built only after that same complete connectivity check at startup.
    for (unsigned slot = 0; slot < 3; ++slot) {
      for (unsigned prior = 0; prior < slot; ++prior) {
        if (element.nodes[slot] == element.nodes[prior]) next.status = BatchStatus::AssemblyFailure;
      }
    }
  }
  return next;
}
TL_T3_HD inline std::uint32_t GatherAssemblyNode(std::size_t node, const AssemblyMemory& memory,
    const ForceTrial* accepted, const ShellSectionLaw* law, const DeviceNodalForceView& forces,
    const double* translation, const double* rotation, AssemblyNode& output) noexcept {
  return mapped_shell::GatherNode<3, AssemblyMemory, ForceTrial, BatchStatus>(
      node, memory, accepted, law, forces, translation, rotation, output);
}
TL_T3_HD inline void PublishAssemblyNode(std::size_t node, const AssemblyNode& value,
    DeviceNodalForceView forces, double* translation, double* rotation) noexcept {
  mapped_shell::PublishNode(node, value, forces, translation, rotation);
}
} // namespace tl::fea::t3::mapped
