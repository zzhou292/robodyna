// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "QbatTypes.h"
#include "lib_src/elements/qeph/QephStartupFrame.h"

namespace tl::fea::qbat::detail {
TL_QBAT_HD inline bool Positive(double x) { return qeph::detail::Positive(x); }
TL_QBAT_HD inline bool Finite(Vec3 x) { return qeph::detail::Finite(x); }

TL_QBAT_HD inline bool Supported(const ResolvedOptions& o) {
  return o.ihbe==11 && o.irep==0 && o.ismstr==2 &&
      o.nptr==2 && o.npts==2 && o.nptt==1 && o.layers==1 &&
      o.idrill==0 && o.npinch==0 && o.material_law==44 && o.property_type==1 &&
      o.ithick==1 && o.iplas==1 && o.offset_ratio==0 &&
      o.inertia_denominator_override==0 &&
      tl::math::Finite(o.membrane_viscosity) && o.membrane_viscosity>=0 &&
      tl::math::Finite(o.numerical_viscosity) && o.numerical_viscosity>=0;
}

TL_QBAT_HD inline bool Finite(const Geometry& g) {
  if (!Positive(g.area_m2) || !Positive(g.reciprocal_area_per_m2) ||
      !tl::math::Finite(g.actual_warpage_m) || !qeph::detail::Proper(g.frame)) return false;
  for (const auto& p:g.centered_projected_position_m) {
    if (!Finite(p)) return false;
  }
  for (double x:g.native_vcore) {
    if (!tl::math::Finite(x)) return false;
  }
  for (const auto& p:g.point) {
    if (!Positive(p.jacobian_m2) || !tl::math::Finite(p.hx_per_m) ||
        !tl::math::Finite(p.hy_per_m)) return false;
    for (double x:p.membrane_b_per_m) {
      if (!tl::math::Finite(x)) return false;
    }
  }
  for (double x:g.assumed_shear_per_m) {
    if (!tl::math::Finite(x)) return false;
  }
  return true;
}
} // namespace tl::fea::qbat::detail
