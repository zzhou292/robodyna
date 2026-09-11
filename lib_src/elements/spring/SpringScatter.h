// SPDX-License-Identifier: AGPL-3.0-or-later
// R4CUM3 current-frame projection, including both finite-length shear arms.
#pragma once
#include "../../math/Fixed3Operations.h"
#if defined(__CUDACC__)
#define TL_SPRING_HD __host__ __device__
#else
#define TL_SPRING_HD
#endif

namespace tl::fea::spring {
struct WrenchValues {
  tl::math::Vec3 force{};
  tl::math::Vec3 couple{};
};

// Input/output values use internally consistent force/moment/length units.
TL_SPRING_HD inline bool Scatter(const tl::math::Matrix3& axes, double length,
    tl::math::Vec3 force, tl::math::Vec3 couple, WrenchValues (&endpoints)[2]) {
  using tl::math::Vec3;
  const double arm = .5 * length;
  const Vec3 first_couple{couple.x, couple.y - arm * force.z,
                          couple.z + arm * force.y};
  const Vec3 second_couple{couple.x, couple.y + arm * force.z,
                           couple.z - arm * force.y};
  endpoints[0] = {tl::math::fixed3::ToWorld(axes, force),
                   tl::math::fixed3::ToWorld(axes, first_couple)};
  endpoints[1] = {tl::math::fixed3::Scale(endpoints[0].force, -1),
                   tl::math::fixed3::Scale(
                       tl::math::fixed3::ToWorld(axes, second_couple), -1)};
  for (const auto& endpoint : endpoints) {
    if (!tl::math::fixed3::Finite(endpoint.force) ||
        !tl::math::fixed3::Finite(endpoint.couple)) {
      return false;
    }
  }
  return true;
}
} // namespace tl::fea::spring
#undef TL_SPRING_HD
