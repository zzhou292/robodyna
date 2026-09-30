// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Checks.h"
#include "Type45Units.h"

namespace tl::fea::type45::detail {
// Selected per-joint JOINT_BLOCK_STIFFNESS branch. Context validation and
// source-role association are performed by Reference::Prepare before this call.
TL_TYPE45_HD inline bool AutomaticCoefficients(
    const Property& property, const GeometryInput& geometry,
    const AutomaticStiffnessContext& context, AutomaticStiffness& output) {
  const auto units=Floors(property.working_units);
  const auto d=Subtract(geometry.position_m[0],geometry.position_m[1]);
  const auto d0=Subtract(geometry.position_m[0],context.main[0].position_m);
  const auto d1=Subtract(geometry.position_m[1],context.main[1].position_m);
  const double xl=Dot(d,d), xx0=Dot(d0,d0), xx1=Dot(d1,d1);
  if (!Finite(xl) || !Finite(xx0) || !Finite(xx1)) return false;
  const double xx=::fmax(xx0,xx1), lever=xx+xl;
  const double denominator=double(4.1f)*context.target_dt_s*context.target_dt_s;
  if (!Finite(lever) || !Positive(denominator)) return false;
  double kt[2]{}, kr[2]{};
  for (unsigned i=0; i<2; ++i) {
    const auto& node=context.main[i];
    const double k1=2*node.mass_kg/denominator-node.translational_stiffness_n_m;
    double k2=units.stiffness_sentinel;
    if (node.inertia_kg_m2>0) {
      kr[i]=node.inertia_kg_m2/denominator-node.rotational_stiffness_nm;
      k2=double(.8f)*kr[i]/::fmax(units.squared_length,lever);
    }
    if (!Finite(k1) || !Finite(k2) || !Finite(kr[i])) return false;
    kt[i]=::fmin(k1,k2);
  }
  AutomaticStiffness next;
  next.unconstrained_translation_limit_n_m=::fmin(kt[0],kt[1]);
  next.unconstrained_rotation_limit_nm=::fmin(kr[0],kr[1]);
  double k=::fmax(next.unconstrained_translation_limit_n_m,
                 ::fmax(2*context.main[0].translational_stiffness_n_m,
                        2*context.main[1].translational_stiffness_n_m));
  double r=next.unconstrained_rotation_limit_nm;
  next.raised_to_structural_stiffness=
    k-next.unconstrained_translation_limit_n_m>units.stiffness_warning;
  if (context.main[0].inertia_kg_m2==0 || context.main[1].inertia_kg_m2==0) {
    r=0;
    next.unconstrained_rotation_limit_nm=0;
    next.raised_to_structural_stiffness=false;
  } else {
    r=::fmax(r,::fmax(2*context.main[0].rotational_stiffness_nm,
                     2*context.main[1].rotational_stiffness_nm));
  }
  if (next.raised_to_structural_stiffness) {
    const double raised=double(1.3f)*k*lever;
    if (!Finite(raised)) return false;
    r=::fmax(r,raised);
  }
  next.blocked_translation_n_m=property.automatic_stiffness_scale*k;
  next.blocked_rotation_nm=property.automatic_stiffness_scale*r;
  if (!Nonnegative(next.blocked_translation_n_m) ||
      !Nonnegative(next.blocked_rotation_nm)) return false;
  output=next;
  return true;
}
} // namespace tl::fea::type45::detail
