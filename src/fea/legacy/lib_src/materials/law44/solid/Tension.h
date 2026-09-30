// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law44/solid/Prepare.h"

namespace tl::material::law44::solid::detail {
// Preserve SIGEPS44's own four-iteration branch, including its conditional mean
// addition. This approximate strain measure is not a general eigensolver.
TL_LAW44_SOLID_HD inline bool TensionFactor(const double (&e)[6], double& result) noexcept {
  const double mean = (e[0] + e[1] + e[2]) * (1. / 3.);
  const double e1 = e[0] - mean, e2 = e[1] - mean, e3 = e[2] - mean;
  const double e4 = .5 * e[3], e5 = .5 * e[4], e6 = .5 * e[5];
  const double e42 = e4*e4, e52 = e5*e5, e62 = e6*e6;
  const double c = -e1*e1 - e2*e2 - e3*e3 - e42 - e52 - e62;
  const double d = -e1*e2*e3 + e1*e52 + e2*e62 + e3*e42 - 2*e4*e5*e6;
  double epst = ::sqrt(-(c * (1. / 3.)));
  const double residual = (epst*epst + c) * epst + d;
  if (!tl::math::Finite(c) || !tl::math::Finite(d) || !tl::math::Finite(residual)) return false;
  if (::fabs(residual) > 1e-8) {
    epst = 1.75 * epst;
    for (unsigned i = 0; i < 4; ++i) {
      const double square = epst * epst;
      const double y = (square + c) * epst + d;
      const double derivative = 3 * square + c;
      if (!tl::math::Finite(y) || !tl::math::Finite(derivative)) return false;
      if (derivative != 0) epst = epst - y / derivative;
    }
    epst = epst + mean;
  }
  const double first = NativeInfinity(), second = 2 * first;
  const double factor = (second - epst) / (second - first);
  if (!tl::math::Finite(epst) || !tl::math::Finite(factor)) return false;
  result = ::fmax(0., ::fmin(1., factor));
  return true;
}
}  // namespace tl::material::law44::solid::detail
