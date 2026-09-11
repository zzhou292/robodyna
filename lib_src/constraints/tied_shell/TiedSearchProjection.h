// SPDX-License-Identifier: AGPL-3.0-or-later
// I2BAR3/I7LIN3: OpenRadioss Copyright (C) 2026 Siemens.
#pragma once
#include "TiedSearchTypes.h"

namespace tl::constraints::tied_shell::search_detail {
namespace v=tl::math::fixed3;
struct FanProjection { double lb=0,lc=0,penetration=0; };
// All arguments and dimensional regularizers here use native working length.
TL_TIED_PATCH_HD inline bool EdgeDistance(Vec3 point,Vec3 a,Vec3 b,double& alpha,double& distance) {
  const auto ab=v::Subtract(b,a);
  alpha=v::Dot(v::Subtract(point,a),ab);
  const double length_squared=v::Dot(ab,ab);
  if (!tl::math::Finite(alpha) || !tl::math::Finite(length_squared)) return false;
  alpha=alpha/::fmax(1e-20,length_squared);
  if (!tl::math::Finite(alpha)) return false;
  alpha=::fmax(0.,alpha);
  alpha=::fmin(1.,alpha);
  const auto projected=v::Add(a,v::Scale(ab,alpha));
  distance=v::Norm(v::Subtract(point,projected));
  return v::Finite(projected) && tl::math::Finite(distance);
}
TL_TIED_PATCH_HD inline bool ProjectFan(Vec3 point,Vec3 a,Vec3 b,Vec3 c,
    double gap,bool triangle,FanProjection& output) {
  auto normal=v::Cross(v::Subtract(b,a),v::Subtract(c,a));
  const double normal_length=v::Norm(normal);
  if (!v::Finite(normal) || !tl::math::Finite(normal_length)) return false;
  const double area=::fmax(1e-20,normal_length);
  normal=v::Divide(normal,area);
  double distance=v::Dot(normal,v::Subtract(point,a));
  const auto projected=v::Subtract(point,v::Scale(normal,distance));
  if (!tl::math::Finite(distance) || !v::Finite(projected)) return false;
  const auto pa=v::Subtract(a,projected),pb=v::Subtract(b,projected),pc=v::Subtract(c,projected);
  FanProjection next;
  next.lb=v::Dot(normal,v::Cross(pc,pa))/area;
  next.lc=v::Dot(normal,v::Cross(pa,pb))/area;
  if (!tl::math::Finite(next.lb) || !tl::math::Finite(next.lc)) return false;
  double alpha=0;
  if (1.-next.lb-next.lc<0) {
    if (!EdgeDistance(point,b,c,alpha,distance)) return false;
  } else if (next.lb<0) {
    if (!EdgeDistance(point,c,a,alpha,distance)) return false;
    if (!triangle) { next.lc=1.-alpha; next.lb=0; }
  } else if (next.lc<0) {
    if (!EdgeDistance(point,a,b,alpha,distance)) return false;
    if (!triangle) { next.lb=alpha; next.lc=0; }
  } else if (distance<0) {
    distance=-distance;
  }
  next.penetration=::fmax(0.,gap-distance);
  output=next;
  return true;
}
} // namespace tl::constraints::tied_shell::search_detail
