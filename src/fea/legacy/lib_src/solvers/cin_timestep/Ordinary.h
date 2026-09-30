// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NodalCinStructuralStep.h"
#include <limits>

#if defined(__CUDACC__)
#define TL_CIN_DT_HD __host__ __device__
#else
#define TL_CIN_DT_HD
#endif
namespace tl::fea::cin_timestep {
struct ScalarLimit {
  double dt = std::numeric_limits<double>::max();
  bool bounded = false;
};
// DTNODA selected non-scaling operation order, for an actual independent DOF.
// Zero stiffness contributes no bound; a free DOF still needs positive M/J.
TL_CIN_DT_HD inline bool OrdinaryLimit(double coefficient, double stiffness,
    double factor, ScalarLimit& output) noexcept {
  if (!std::isfinite(coefficient) || coefficient <= 0 ||
      !std::isfinite(stiffness) || stiffness < 0 ||
      !std::isfinite(factor) || factor <= 0 || factor > 1) return false;
  ScalarLimit next;
  if (stiffness > 0) {
    next.dt = factor*::sqrt(2*coefficient/stiffness);
    if (!std::isfinite(next.dt) || next.dt <= 0) return false;
    next.bounded = true;
  }
  output = next;
  return true;
}
} // namespace tl::fea::cin_timestep
#undef TL_CIN_DT_HD
