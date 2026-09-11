// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include <cstddef>

namespace tl::material::law90 {
namespace relocation_detail {
TL_LAW90_HD inline bool Range(const void* pointer, std::size_t bytes,
                              std::size_t alignment) noexcept {
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  return pointer && address % alignment == 0 && bytes <= UINTPTR_MAX - address;
}
TL_LAW90_HD inline bool Disjoint(const void* a, std::size_t a_bytes,
                                 const void* b, std::size_t b_bytes) noexcept {
  const auto x = reinterpret_cast<std::uintptr_t>(a);
  const auto y = reinterpret_cast<std::uintptr_t>(b);
  return x <= y ? a_bytes <= y - x : b_bytes <= x - y;
}
} // namespace relocation_detail
// Relocate already prepared values to an immutable byte-identical curve copy.
// The caller owns/certifies the copy and its lifetime, including host-to-device
// transfer. New addresses are checked but never dereferenced here. This is not
// preparation, curve validation, or permission to replace curve values.
// Source, output and the two destination arrays must be mutually disjoint.
TL_LAW90_HD inline Status RelocatePreparedCurve(const PreparedMaterial& source,
    CurveView curve, PreparedMaterial& output) noexcept {
  using namespace relocation_detail;
  if (!source.initialized() || curve.count != source.curve().count ||
      curve.count < 2 || curve.count > 1024) return Status::InvalidInput;
  const std::size_t bytes = sizeof(double) * curve.count;
  if (!Range(curve.compression_strain, bytes, alignof(double)) ||
      !Range(curve.stress_pa, bytes, alignof(double)) ||
      !Disjoint(curve.compression_strain, bytes, curve.stress_pa, bytes) ||
      !Disjoint(&source, sizeof(source), &output, sizeof(output)) ||
      !Disjoint(curve.compression_strain, bytes, &source, sizeof(source)) ||
      !Disjoint(curve.stress_pa, bytes, &source, sizeof(source)) ||
      !Disjoint(curve.compression_strain, bytes, &output, sizeof(output)) ||
      !Disjoint(curve.stress_pa, bytes, &output, sizeof(output))) return Status::InvalidInput;
  auto next = source;
  next.curve_ = curve;
  output = next;
  return Status::Ok;
}
} // namespace tl::material::law90
