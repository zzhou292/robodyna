// SPDX-License-Identifier: AGPL-3.0-or-later
// VINTER2 directional search and binary64 interpolation, including extrapolation.
#pragma once
#include "Type13Units.h"

namespace tl::fea::type13::detail {
TL_TYPE13_HD inline Status Interpolate(const Curve& curve, double x,
                                      unsigned& position, double& y) {
  if (!tl::math::fixed3::Finite(x) || position >= CurvePoints - 1) {
    return Status::InvalidInput;
  }
  unsigned next = position;
  // Native ILEN is fixed at entry, whereas IPOS changes during the loop.
  const unsigned remaining = CurvePoints - 1 - position;
  for (unsigned iteration = 1;; ++iteration) {
    if (iteration <= remaining - 1 && x > curve.points[next + 1].x) {
      ++next;
    } else if (next >= 1 && x < curve.points[next].x) {
      --next;
    } else {
      break;
    }
  }
  const auto a = curve.points[next];
  const auto b = curve.points[next + 1];
  const double slope = (b.y - a.y) / (b.x - a.x);
  const double value = a.y + slope * (x - a.x);
  if (!tl::math::fixed3::Finite(slope) || !tl::math::fixed3::Finite(value)) {
    return Status::NonfiniteResult;
  }
  position = next;
  y = value;
  return Status::Success;
}
} // namespace tl::fea::type13::detail
