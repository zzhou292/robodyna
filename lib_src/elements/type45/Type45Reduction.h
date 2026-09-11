// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45Response.h"

namespace tl::fea::type45::detail {
// Complete selected RDTIME33 followed by RCUM33 endpoint output. No added
// mass or automatic scaling branch is selected by this value profile.
TL_TYPE45_HD inline bool Reduce(const Reference& reference,
    const HistoryValues& next, const Diagnostics& diagnostics,
    EndpointResult (&endpoint)[2]) {
  const auto units=Floors(reference.property().working_units);
  double mass=0, inertia=0;
  if (!Harmonic(reference.damping(0).mass_kg,reference.damping(1).mass_kg,units.mass20,mass) ||
      !Harmonic(reference.damping(0).mean_principal_inertia_kg_m2,
                reference.damping(1).mean_principal_inertia_kg_m2,units.inertia20,inertia)) return false;
  double kt=diagnostics.maximum_stiffness_n_m;
  double kr=diagnostics.maximum_rotational_stiffness_nm;
  const double ct=diagnostics.damping_n_s_m, cr=diagnostics.rotational_damping_nm_s;
  if (mass>units.mass15) {
    const double a=ct+::sqrt(ct*ct+kt*mass);
    kt=(a*a)/mass;
  }
  if (inertia>units.inertia15) {
    const double a=cr+::sqrt(cr*cr+kr*inertia);
    kr=(a*a)/inertia;
  }
  const Vec3 local_lever=Scale(Cross(diagnostics.local_separation_m,next.local_force_n),.5);
  endpoint[0].force_n=ToWorld(next.frame,next.local_force_n);
  endpoint[1].force_n=Scale(endpoint[0].force_n,-1);
  endpoint[0].couple_nm=ToWorld(next.frame,Add(next.local_couple_nm,local_lever));
  endpoint[1].couple_nm=Scale(ToWorld(next.frame,Subtract(next.local_couple_nm,local_lever)),-1);
  const double xx=Dot(diagnostics.local_separation_m,diagnostics.local_separation_m);
  for (unsigned i=0; i<2; ++i) {
    endpoint[i].translational_stiffness_n_m=2*kt;
    endpoint[i].rotational_stiffness_nm=kr+kt*xx;
    if (!Finite(endpoint[i].force_n) || !Finite(endpoint[i].couple_nm) ||
        !Nonnegative(endpoint[i].translational_stiffness_n_m) ||
        !Nonnegative(endpoint[i].rotational_stiffness_nm)) return false;
  }
  return true;
}
} // namespace tl::fea::type45::detail
