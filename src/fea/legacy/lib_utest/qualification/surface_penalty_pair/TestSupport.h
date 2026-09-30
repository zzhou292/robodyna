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
  double k = 32;
  TL_SURFACE_HD ct::SurfacePenaltyInput input() const {
    return {{x, 8, 3, 1}, {v, 8, 3, 1}, a, b, k};
  }
};
} // namespace pair_test
