#pragma once
#include "RowValues.h"
#include <cstddef>
#include <cstdint>

namespace tl::fea::nodal_seal {
inline constexpr std::uint32_t RowThreads = 256;
inline constexpr std::uint32_t MaxRowBlocks = 256;
inline constexpr std::size_t MaxRowBytes = MaxRowBlocks * sizeof(RowSummary);
static_assert(MaxRowBytes == 8192);

inline std::uint32_t RowBlocks(std::size_t nodes) noexcept {
  if (!nodes || nodes > UINT32_MAX) return 0;
  const auto count = 1 + (nodes - 1) / RowThreads;
  return count < MaxRowBlocks ? static_cast<std::uint32_t>(count) : MaxRowBlocks;
}
inline std::size_t ControlBytes(std::size_t prefix, std::size_t nodes) noexcept {
  const auto blocks = RowBlocks(nodes);
  const auto tail = blocks * sizeof(RowSummary);
  if (!blocks || prefix % alignof(RowSummary) || prefix > SIZE_MAX - tail) return 0;
  return prefix + tail;
}
struct RowScratch {
  RowSummary* summaries = nullptr;
  std::uint32_t blocks = 0;
};
template<class Control>
inline RowScratch ControlTail(Control* control, std::size_t nodes) noexcept {
  static_assert(sizeof(Control) % alignof(RowSummary) == 0);
  return {reinterpret_cast<RowSummary*>(reinterpret_cast<unsigned char*>(control) + sizeof(Control)),
          RowBlocks(nodes)};
}
} // namespace tl::fea::nodal_seal
