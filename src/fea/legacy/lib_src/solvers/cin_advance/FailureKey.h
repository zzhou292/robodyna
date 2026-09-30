// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../FENodalState.h"
#include <limits>
namespace tl::fea::cin_advance {
// CUDA's native unsigned64 atomicMin overload. The node index is the primary
// order key; each node reports only its first local failure.
using FailureKey = unsigned long long;
inline constexpr FailureKey NoFailure = std::numeric_limits<FailureKey>::max();
static_assert(sizeof(FailureKey) == sizeof(std::uint64_t));
static_assert(sizeof(NodalStatus) <= sizeof(std::uint32_t));
static_assert(static_cast<int>(NodalStatus::Ok) == 0);
static_assert(MaxActiveNodalStateNodes < UINT32_MAX);
TL_SURFACE_HD constexpr FailureKey EncodeFailure(std::uint32_t node, NodalStatus status) noexcept {
  return (FailureKey(node) << 32) | static_cast<std::uint32_t>(status);
}
TL_SURFACE_HD constexpr std::uint32_t FailureNode(FailureKey key) noexcept {
  return static_cast<std::uint32_t>(key >> 32);
}
TL_SURFACE_HD constexpr NodalStatus FailureStatus(FailureKey key) noexcept {
  return static_cast<NodalStatus>(static_cast<std::uint32_t>(key));
}
} // namespace tl::fea::cin_advance
