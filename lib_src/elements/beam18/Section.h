// SPDX-License-Identifier: AGPL-3.0-or-later
// Native HM_READ_PROP18 / DEFBEAM_SECT, ISECT=2, INTR=2.
#pragma once
#include "Units.h"

namespace tl::fea::beam18::detail {
TL_BEAM18_HD inline Status PrepareSection(double radius, Section& output) noexcept {
  if (!Positive(radius)) return Status::InvalidInput;
  Section next{};
  const double pi = ::atan2(0.0,-1.0);
  const double area = pi*radius*radius;
  const double point_area = area/4;
  const double r = radius*::sqrt(2.0)*.5;
  const double dphi = 2*pi/4;
  double phi = dphi*.5;
  // Retain native point visitation and transcendental values; no generic table.
  for (auto& point : next.point) {
    point = {r*::sin(phi),r*::cos(phi),point_area};
    phi = phi+dphi;
    const double own = point.area*point.area*(1.0/12.0);
    next.area = next.area+point.area;
    next.inertia_y = next.inertia_y+own+point.area*point.y*point.y;
    next.inertia_z = next.inertia_z+own+point.area*point.z*point.z;
  }
  next.inertia_x = next.inertia_y+next.inertia_z;
  next.membrane_damping = 0;
  // Converter df=0 is resolved by HM_READ_PROP18 to EM02.
  next.flexural_damping = 1.0/100.0;
  if (!Positive(next.area) || !Positive(next.inertia_y) || !Positive(next.inertia_z) ||
      !Positive(next.inertia_x)) return Status::NonfiniteResult;
  for (const auto& point : next.point)
    if (!tl::math::Finite(point.y) || !tl::math::Finite(point.z) || !Positive(point.area))
      return Status::NonfiniteResult;
  output = next;
  return Status::Success;
}
} // namespace tl::fea::beam18::detail
