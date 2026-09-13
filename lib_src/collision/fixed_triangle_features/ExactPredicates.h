// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../SurfaceContactTypes.h"

namespace tlfea::contact::fixed_triangle_features::exact {

struct Sign {
  int value = 0;
  bool valid = false;
};

// Exact signs for finite binary64 inputs.  The implementation converts every
// coordinate to a common dyadic integer scale and uses fixed-capacity integer
// arithmetic; it allocates no memory and never depends on long-double width.
Sign Orient2D(Vec3 a, Vec3 b, Vec3 c, int dropped_axis) noexcept;
Sign Orient3D(Vec3 a, Vec3 b, Vec3 c, Vec3 d) noexcept;

}  // namespace tlfea::contact::fixed_triangle_features::exact
