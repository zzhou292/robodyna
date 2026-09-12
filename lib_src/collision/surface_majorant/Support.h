// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../SurfaceJacobianMajorantTypes.h"

namespace tlfea::contact::surface_majorant {
template<class T> TL_SURFACE_HD inline std::uint32_t FindNode(
    const T* entries, std::uint32_t count, std::uint32_t node) {
  std::uint32_t i = 0;
  while (i < count && entries[i].node != node) ++i;
  return i;
}
template<class T> TL_SURFACE_HD inline void SortNodes(T* entries, std::uint32_t count) {
  for (std::uint32_t i = 1; i < count; ++i) {
    for (std::uint32_t j = i; j && entries[j].node < entries[j - 1].node; --j) {
      const T previous = entries[j - 1];
      entries[j - 1] = entries[j];
      entries[j] = previous;
    }
  }
}
TL_SURFACE_HD inline Vec3 Project(Vec3 v, std::uint8_t fixed) {
  return {fixed & 1 ? 0.0 : v.x, fixed & 2 ? 0.0 : v.y, fixed & 4 ? 0.0 : v.z};
}
TL_SURFACE_HD inline SurfaceMajorantStatus Check(std::uint32_t node, std::uint32_t count,
    std::uint8_t fixed) {
  if (node >= count) return SurfaceMajorantStatus::OutOfRange;
  return fixed <= 7 ? SurfaceMajorantStatus::Ok : SurfaceMajorantStatus::InvalidInput;
}
TL_SURFACE_HD inline bool RepresentedProduct(double a, double b, double value) {
  return IsFinite(value) && (a == 0 || b == 0 || value != 0);
}
} // namespace tlfea::contact::surface_majorant
