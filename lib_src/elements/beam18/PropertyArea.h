// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss HM_READ_PROP18 / DEFBEAM_SECT, ISECT2: pre-PMASS GEO1.
#pragma once
#include "Units.h"
namespace tl::fea::beam18 {
namespace detail {
// Shared native association; callers retain the same original PI evaluation.
TL_BEAM18_HD inline double CircularArea(double radius, double pi) noexcept {
  return pi*radius*radius;
}
}
// Native working-length input and squared-working-length output. This is the
// property reader AREA, not the later sum of integration-point areas. Source
// radius/profile/units binding belongs to the prepared reference's caller.
TL_BEAM18_HD inline Status EvaluateCircularPropertyArea(double radius, double* output) noexcept {
  if (!output || !detail::Positive(radius)) return Status::InvalidInput;
  const double pi = ::atan2(0.0, -1.0);
  const double area = detail::CircularArea(radius, pi);
  if (!detail::Positive(area)) return Status::NonfiniteResult;
  *output = area;
  return Status::Success;
}
}
