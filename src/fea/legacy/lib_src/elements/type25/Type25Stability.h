// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete donor hashes: lib_utest/qualification/type25/native/source-manifest.json.
// GNU AGPL v3 or later; see LICENSE.md alongside.
#pragma once
#include "Type25Property.h"

#if defined(__CUDACC__)
#define TL_TYPE25_HD __host__ __device__
#else
#define TL_TYPE25_HD
#endif

namespace tl::fea::type25 {
struct Stability { double critical_dt_s=0,translation_stiffness_N_per_m=0,rotation_stiffness_Nm_per_rad=0; };
// R6DEF3 + R2LEN3 unscaled elementary dt for Ileng=0. This intentionally uses
// the source property M/J, not the combined live nodal coefficients. Native
// EM15 arithmetic runs in the explicitly declared working units; only the
// result is converted to SI. No guessed dimensional interpretation of floors.
TL_TYPE25_HD inline Status CriticalStep(SourceUnits units,const Property& p,double length_m,Stability& output) {
  detail::Units u;
  if(!detail::ResolveUnits(units,u)||!ValidProperty(p)||!detail::Positive(length_m))return Status::InvalidInput;
  const double length=length_m/units.length_to_m;
  double k[4],c[4];
  for(unsigned i=0;i<4;++i) {
    k[i]=p.stiffness[i]/(i<2?u.translation_stiffness:u.rotation_stiffness);
    c[i]=p.damping[i]/(i<2?u.translation_damping:u.rotation_damping);
    if(!detail::Positive(k[i])||!detail::Nonnegative(c[i]))return Status::NonfiniteResult;
  }
  double mass=p.mass_kg/u.mass,inertia=p.isotropic_inertia_kg_m2/u.inertia;
  double kt=::fmax(k[0],k[1]),kr=::fmax(k[2],k[3])+k[1]*length*length;
  const double ct=::fmax(c[0],c[1]),cr=::fmax(c[2],c[3])+c[1]*length*length;
  if(!detail::Positive(length)||!detail::Positive(mass)||!detail::Positive(inertia)||
     !detail::Positive(kr)||!detail::Nonnegative(cr))return Status::NonfiniteResult;
  Stability result;result.translation_stiffness_N_per_m=kt*u.translation_stiffness;
  result.rotation_stiffness_Nm_per_rad=kr*u.rotation_stiffness;
  if(ct+kt<1e-15)mass=1;
  kt=::fmax(1e-15,kt);
  const double rt=::sqrt(ct*ct+mass*kt)+ct;
  const double translation=mass/::fmax(1e-15,rt);
  if(cr+kr<1e-15)inertia=1;
  kr=::fmax(1e-15,kr);
  const double rr=::sqrt(cr*cr+inertia*kr)+cr;
  const double rotation=inertia/::fmax(1e-15,rr);
  result.critical_dt_s=::fmin(translation,rotation)*units.time_to_s;
  if(!detail::Positive(rt)||!detail::Positive(rr)||!detail::Positive(translation)||!detail::Positive(rotation)||
     !detail::Positive(result.critical_dt_s)||!detail::Positive(result.translation_stiffness_N_per_m)||
     !detail::Positive(result.rotation_stiffness_Nm_per_rad))return Status::NonfiniteResult;
  output=result;return Status::Success;
}
} // namespace tl::fea::type25

#undef TL_TYPE25_HD
