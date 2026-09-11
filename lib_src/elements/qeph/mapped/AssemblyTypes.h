// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Stiffness.h"
#include "../QephBatch.h"
#include "ObserverTypes.h"
#include "../../mapped_shell/NodeGather.h"
#include <cstdint>

namespace tl::fea::qeph::mapped {
// Each incidence is 4*parent+local_slot. Startup fills each node's range in
// ascending source-parent order; gather preserves the serial scatter sum.
struct AssemblyParent {
  NodalStiffness stiffness;
  BatchStatus status=BatchStatus::Success;
  std::uint32_t node=UINT32_MAX;
};
using AssemblyNode = mapped_shell::AssemblyNode;
struct AssemblyMemory {
  std::uint32_t* offsets=nullptr;
  std::uint32_t* incidence=nullptr;
  AssemblyParent* parent=nullptr;
  AssemblyNode* node=nullptr;
  // Integer diagnostic ordering only; no floating-point atomic accumulation.
  unsigned long long* failure=nullptr;
  ObserverSummary* observer=nullptr;
};
inline constexpr unsigned long long NoAssemblyFailure=mapped_shell::NoAssemblyFailure;
} // namespace tl::fea::qeph::mapped
