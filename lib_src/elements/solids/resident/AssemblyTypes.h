// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>
namespace tl::fea::solids::batch_detail {
struct AssemblyNode { double force[3]{}, translation = 0; };
struct AssemblyMemory {
  std::uint32_t* offsets = nullptr;
  std::uint32_t* incidence = nullptr;
  AssemblyNode* nodes = nullptr;
  std::size_t occurrences = 0, arena_bytes = 0;
  unsigned fallback = 0;
  bool prepared = false;
};
} // namespace tl::fea::solids::batch_detail
