// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Stiffness.h"
#include "../T3Batch.h"
#include "../../mapped_shell/NodeGather.h"

namespace tl::fea::t3::mapped {
struct AssemblyParent {
  NodalStiffness stiffness;
  BatchStatus status = BatchStatus::Success;
  std::uint32_t node = UINT32_MAX;
};
using AssemblyNode = mapped_shell::AssemblyNode;
struct AssemblyMemory {
  std::uint32_t* offsets = nullptr;
  std::uint32_t* incidence = nullptr;
  AssemblyParent* parent = nullptr;
  AssemblyNode* node = nullptr;
  unsigned long long* failure = nullptr;
};
inline constexpr unsigned long long NoAssemblyFailure = mapped_shell::NoAssemblyFailure;
} // namespace tl::fea::t3::mapped
