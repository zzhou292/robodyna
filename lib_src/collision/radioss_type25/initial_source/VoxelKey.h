// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/math/HostDevice.h"
namespace tlfea::contact::radioss_type25::initial_source::detail {
// Internal precondition: native padded extent <=8M, cells in [1,grid+2].
// Exact integer key in existing double sort storage, x varying fastest.
TL_MATH_HOST_DEVICE inline double NativeVoxelKey(const int* grid,int x,int y,int z) {
  const auto nx=static_cast<unsigned long long>(grid[0]+2);
  const auto ny=static_cast<unsigned long long>(grid[1]+2);
  return double((static_cast<unsigned long long>(z-1)*ny+unsigned(y-1))*nx+unsigned(x-1));
}
}
