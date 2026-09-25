// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
namespace tlfea::contact::radioss_type25::search::detail {
// Descriptor admission only; allocation ownership and accessibility remain the
// caller's contract. Check before dereferencing host maps or launching device work.
template<class T>
inline bool Span(const T* data, std::size_t count) noexcept {
  if (!count) return data == nullptr;
  const auto address = reinterpret_cast<std::uintptr_t>(data);
  return data && address % alignof(T) == 0 &&
      count <= SIZE_MAX / sizeof(T) &&
      count * sizeof(T) <= UINTPTR_MAX - address;
}
inline bool VectorSpan(VectorView view, std::size_t nodes,
    std::size_t& bytes) noexcept {
  if (!view.valid() || view.node_count != nodes) return false;
  const auto values = (view.node_count - 1) * view.node_stride +
      2 * view.component_stride + 1;
  if (values > SIZE_MAX || !Span(view.data, static_cast<std::size_t>(values)))
    return false;
  bytes = static_cast<std::size_t>(values) * sizeof(double);
  return true;
}
} // namespace tlfea::contact::radioss_type25::search::detail
