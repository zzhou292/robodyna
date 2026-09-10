// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchBinding.h"

namespace tl::fea::shell_batch_detail {
// Host model preparation shared by the two typed participants. Their one
// selected reference/history remains formulation-specific. The immutable
// producer has already established complete union coverage and native order.
template<class Model>
void ApplyJoinedMass(const ShellBatchBinding& binding,Model& model) noexcept {
  for(std::size_t n=0;n<binding.node_count();++n) {
    const auto& node=binding.nodes()[n];
    model.initial_position[n]=node.position;
    model.mass[n]=node.native.mass; model.inertia[n]=node.native.isotropic_inertia;
    model.physical[n]=node.native.physical_inertia; model.added[n]=node.native.added_inertia;
  }
  model.joined=true;
}
} // namespace tl::fea::shell_batch_detail
