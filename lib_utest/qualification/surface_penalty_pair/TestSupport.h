#pragma once
#include "lib_src/collision/penalty_pair/Values.h"

namespace pair_test {
namespace ct = tlfea::contact;
struct Case {
  double x[24]{1, 1, .5, -1, 1, .5, -1, -1, .5, 1, -1, .5,
               1, 1, 0, -1, 1, 0, -1, -1, 0, 1, -1, 0};
  double v[24]{1, 2, 3, -2, 1, 4, 2, -1, 5, -1, -2, 6,
               3, 2, -1, 1, -3, 2, 2, 1, -2, 0, 3, -3};
  ct::SurfacePenaltyEndpoint a{{{0, 1, 2, 3}, {.25, .25, .25, .25}, 4}, {}, .375};
  ct::SurfacePenaltyEndpoint b{{{4, 5, 6, 7}, {.25, .25, .25, .25}, 4}, {}, .375};
  double gap = -.25;
  double k = 32;
  TL_SURFACE_HD ct::SurfacePenaltyInput input() const {
    return {{x, 8, 3, 1}, {v, 8, 3, 1}, a, b, gap, k};
  }
};
inline bool RefreshGap(Case& value) {
  const auto input = value.input();
  ct::Vec3 a, b;
  if (ct::EvaluateWeightedSurfacePosition(input.positions, value.a.point, &a) != ct::Status::kOk ||
      ct::EvaluateWeightedSurfacePosition(input.positions, value.b.point, &b) != ct::Status::kOk)
    return false;
  value.gap = (ct::geometry_detail::Length(ct::Subtract(a, b)) - value.a.reference_half_thickness_m) -
              value.b.reference_half_thickness_m;
  return true;
}
} // namespace pair_test
