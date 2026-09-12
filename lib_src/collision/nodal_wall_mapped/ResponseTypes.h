// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>

namespace tlfea::contact::nodal_wall_mapped::response {
inline constexpr unsigned Threads = 128;
inline constexpr unsigned MaximumBlocks = 4096;
inline constexpr unsigned long long NoFailure = ~0ull;
inline constexpr unsigned Blocks(std::size_t nodes) noexcept {
  const auto count = nodes ? 1+(nodes-1)/Threads : 0;
  return static_cast<unsigned>(count < MaximumBlocks ? count : MaximumBlocks);
}
struct Scratch {
  // Immutable CSR, ordered by compact contact row within each rigid group.
  std::uint32_t* offsets = nullptr;
  std::uint32_t* rows = nullptr;
  double* maxima = nullptr;
  std::size_t node_count = 0;
  std::size_t block_count = 0;
};
} // namespace tlfea::contact::nodal_wall_mapped::response
