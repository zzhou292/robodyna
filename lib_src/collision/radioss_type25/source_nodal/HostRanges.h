// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstddef>
#include <cstdint>
namespace tlfea::contact::radioss_type25::source_nodal::detail {
// Descriptor-only range utility shared by the two host startup value adapters.
// Canonical empty spans and pointer accessibility are admitted by their caller.
struct Range {
  const void* data;
  std::size_t bytes;
};
inline bool Disjoint(Range a, Range b) noexcept {
  if (!a.bytes || !b.bytes) return true;
  const auto first = reinterpret_cast<std::uintptr_t>(a.data);
  const auto second = reinterpret_cast<std::uintptr_t>(b.data);
  return a.data && b.data && a.bytes <= UINTPTR_MAX - first && b.bytes <= UINTPTR_MAX - second &&
      (first + a.bytes <= second || second + b.bytes <= first);
}
}
