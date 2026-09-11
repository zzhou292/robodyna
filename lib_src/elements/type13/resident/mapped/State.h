// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../../../assembly/ShellPhysicalBinding.h"
#include "../../../../constraints/NodalRigidAssemblyBinding.h"
#include "../../../../solvers/NodalCinRuntime.h"

namespace tl::fea::type13::mapped {
struct State {
  State(const ShellPhysicalBinding& p, const NodalRigidAssemblyBinding& r,
        FENodalState& o, std::size_t witnesses)
      : physical(p), rigid(r), owner(&o), witness_count(witnesses) {}
  ShellPhysicalBinding physical;
  NodalRigidAssemblyBinding rigid;
  FENodalState* owner;
  std::size_t witness_count;
  NodalAssemblyView initial_sources;
};
} // namespace tl::fea::type13::mapped
