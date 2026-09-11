// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Stiffness.h"
#include "../QephBatch.h"
#include <cstdint>

namespace tl::fea::qeph::mapped {
// Each incidence is 4*parent+local_slot. Startup fills each node's range in
// ascending source-parent order; gather preserves the serial scatter sum.
struct AssemblyParent {
  NodalStiffness stiffness;
  BatchStatus status=BatchStatus::Success;
  std::uint32_t node=UINT32_MAX;
};
struct AssemblyNode {
  double value[8]{}; // force XYZ, couple XYZ, translation STI, rotation STIR
  bool touched=false;
};
struct AssemblyMemory {
  std::uint32_t* offsets=nullptr;
  std::uint32_t* incidence=nullptr;
  AssemblyParent* parent=nullptr;
  AssemblyNode* node=nullptr;
  // Integer diagnostic ordering only; no floating-point atomic accumulation.
  unsigned long long* failure=nullptr;
};
inline constexpr unsigned long long NoAssemblyFailure=~0ull;
} // namespace tl::fea::qeph::mapped
