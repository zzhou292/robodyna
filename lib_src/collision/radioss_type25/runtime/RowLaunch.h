// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>

namespace tlfea::contact::radioss_type25::runtime_detail {
// Each row owns its scratch and ordered history. Give the scheduler independent
// row blocks instead of serializing long row ranges behind 256 resident blocks.
// The portable grid bound also keeps blockDim.x*gridDim.x within unsigned int.
inline constexpr unsigned RowThreads = 128;
inline constexpr unsigned MaximumRowBlocks = 65535;
inline constexpr unsigned IndependentRowBlocks(std::size_t rows) noexcept {
  const auto blocks = rows / RowThreads + (rows % RowThreads != 0);
  return blocks == 0 ? 1 : blocks > MaximumRowBlocks ? MaximumRowBlocks :
      static_cast<unsigned>(blocks);
}
} // namespace tlfea::contact::radioss_type25::runtime_detail
