// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected SORDEFT3 arithmetic, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "BrickFrame.h"

namespace tl::fea::solid_common {

// Row-major nonsymmetric displacement gradient. Native SZTORTH3 ISORTH0
// supplies the current frame columns as the three material directions.
TL_BRICK_HD inline bool MaterialGradient(const Matrix3& frame,
    const double (&world_gradient)[9], double (&material_gradient)[9]) noexcept {
  for (double value : frame.v) {
    if (!tl::math::Finite(value)) return false;
  }
  for (double value : world_gradient) {
    if (!tl::math::Finite(value)) return false;
  }
  double next[9];
  for (unsigned row = 0; row < 3; ++row) {
    const double gx = frame.v[row];
    const double gy = frame.v[3+row];
    const double gz = frame.v[6+row];
    const double sx = world_gradient[0]*gx+world_gradient[3]*gy+world_gradient[6]*gz;
    const double sy = world_gradient[1]*gx+world_gradient[4]*gy+world_gradient[7]*gz;
    const double sz = world_gradient[2]*gx+world_gradient[5]*gy+world_gradient[8]*gz;
    for (unsigned column = 0; column < 3; ++column) {
      next[3*row+column] = sx*frame.v[column]+sy*frame.v[3+column]+sz*frame.v[6+column];
      if (!tl::math::Finite(next[3*row+column])) return false;
    }
  }
  for (unsigned k = 0; k < 9; ++k) material_gradient[k] = next[k];
  return true;
}
}  // namespace tl::fea::solid_common
