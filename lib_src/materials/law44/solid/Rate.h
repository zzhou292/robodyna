// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law44/solid/Prepare.h"

namespace tl::material::law44::solid::detail {
TL_LAW44_SOLID_HD inline bool FilterRate(const Parameters& p, const History& h,
    const Input& in, double& filtered, double& factor) noexcept {
  // MSTRAIN_RATE IDEV0: engineering shear is halved before the tensor norm.
  const auto& r = in.engineering_rate_per_s;
  const double e4 = .5 * r[3], e5 = .5 * r[4], e6 = .5 * r[5];
  const double square = r[0]*r[0] + r[1]*r[1] + r[2]*r[2] +
                        2 * (e4*e4 + e5*e5 + e6*e6);
  const double argument = p.angular_cutoff_per_s * in.dt_s;
  if (!tl::math::Finite(square) || !tl::math::Finite(argument)) return false;
  const double alpha = ::fmin(1., argument);
  filtered = alpha * ::sqrt(square) + (1 - alpha) * h.filtered_rate_per_s;
  factor = 1 + ::pow(p.inverse_rate_c * filtered, p.inverse_rate_p);
  return tl::math::Finite(filtered) && tl::math::Finite(factor);
}
}  // namespace tl::material::law44::solid::detail
