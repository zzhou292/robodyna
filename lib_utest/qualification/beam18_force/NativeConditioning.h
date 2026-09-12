// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NativeOracle.h"

namespace beam18_force_test {
// Independent native section coordinates, areas and returned stresses give
// dimensional absolute-sum scales for cancellation. These are roundoff terms,
// not a replacement for the component-relative comparison. No production
// section/resultant helper participates in this calculation.
inline void SetCancellationScales(NativeResult& result, double length_unit_m) {
  double force_sum = 0, moment_sum = 0;
  for (unsigned p = 0; p < 4; ++p) {
    const double y = result.next.reference[3*p] * length_unit_m;
    const double z = result.next.reference[3*p+1] * length_unit_m;
    const double area = result.next.reference[3*p+2] * length_unit_m * length_unit_m;
    const double fx = area * std::abs(result.si[7*p]);
    const double fy = area * std::abs(result.si[7*p+1]);
    const double fz = area * std::abs(result.si[7*p+2]);
    force_sum += fx + fy + fz;
    moment_sum += fy*std::abs(z) + fz*std::abs(y) + fx*(std::abs(y)+std::abs(z));
  }
  double damped_force_sum = force_sum, damped_moment_sum = moment_sum;
  for (unsigned k = 0; k < 3; ++k) {
    damped_force_sum += std::abs(result.si[72+k]);
    damped_moment_sum += std::abs(result.si[75+k]);
  }
  const auto set = [&](unsigned begin, unsigned end, double scale) {
    for (unsigned i = begin; i < end; ++i) result.cancellation_scale[i] = scale;
  };
  set(28,31,1.); set(53,62,1.); // Unit frame vectors, including near-zero components.
  set(31,34,force_sum); set(34,37,moment_sum);
  set(72,75,damped_force_sum); set(75,78,damped_moment_sum);
  set(41,47,damped_force_sum);
  set(47,53,damped_moment_sum + .5*result.si[62]*damped_force_sum);
}
} // namespace beam18_force_test
