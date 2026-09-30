// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SGCOOR3/S8EDEFOT3: OpenRadioss (C) 2026 Siemens.
#pragma once
#include "KinematicsTypes.h"
#include "lib_src/elements/solid18/Solid18Orientation.h"
#include "lib_src/elements/solid_common/FrameTensor.h"
namespace tl::fea::solid18::total_strain::detail {
TL_SOLID18_HD inline Status TotalGradient(const Reference& reference,
    const Vec3 (&native)[8], Kinematics& result) noexcept {
  const auto& saved = reference.coefficients().source_relative_position_m;
  auto& displacement = result.reference_displacement_m;
  for (unsigned n = 0; n < 7; ++n) {
    displacement[n].x = native[n].x-native[7].x-saved[n].x;
    displacement[n].y = native[n].y-native[7].y-saved[n].y;
    displacement[n].z = native[n].z-native[7].z-saved[n].z;
    if (!solid18::detail::Finite(displacement[n])) return Status::NonfiniteResult;
  }
  displacement[7] = {0,0,0};
  for (unsigned ip = 0; ip < 8; ++ip) {
    const auto& pij = reference.coefficients().point_pij_per_m[ip];
    auto& point = result.point[ip];
    auto& world = point.world_displacement_gradient;
    for (unsigned row = 0; row < 3; ++row) {
      for (unsigned column = 0; column < 3; ++column) {
        double sum = pij[column]*solid18::detail::Component(displacement[0],row);
        for (unsigned n = 1; n < 8; ++n)
          sum += pij[3*n+column]*solid18::detail::Component(displacement[n],row);
        world[3*row+column] = sum;
      }
    }
    if (!solid_common::MaterialGradient(result.geometry.frame, world,
                                       point.material_displacement_gradient))
      return Status::NonfiniteResult;
  }
  return Status::Success;
}
}  // namespace tl::fea::solid18::total_strain::detail
