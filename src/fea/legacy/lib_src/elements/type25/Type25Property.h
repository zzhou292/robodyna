// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Complete donor hashes: lib_utest/qualification/type25/native/source-manifest.json.
// GNU AGPL v3 or later; see LICENSE.md alongside.
#pragma once
#include "Type25Types.h"
#include "../../math/Fixed3Operations.h"

#if defined(__CUDACC__)
#define TL_TYPE25_HD __host__ __device__
#else
#define TL_TYPE25_HD
#endif

namespace tl::fea::type25 {
namespace detail {
TL_TYPE25_HD inline bool Positive(double x) { return tl::math::fixed3::Finite(x)&&x>0; }
TL_TYPE25_HD inline bool Nonnegative(double x) { return tl::math::fixed3::Finite(x)&&x>=0; }
// Source-unit factors used solely to express the donor's regularized arithmetic.
struct Units {
  double mass=0,inertia=0,translation_stiffness=0,rotation_stiffness=0;
  double translation_damping=0,rotation_damping=0,length_floor=0,shear_length_floor=0,time=0;
};
TL_TYPE25_HD inline bool ResolveUnits(SourceUnits input,Units& out) {
  if(!Positive(input.mass_to_kg)||!Positive(input.length_to_m)||!Positive(input.time_to_s))return false;
  const double t2=input.time_to_s*input.time_to_s;
  Units u;u.mass=input.mass_to_kg;u.inertia=input.mass_to_kg*input.length_to_m*input.length_to_m;
  u.translation_stiffness=u.mass/t2;u.rotation_stiffness=u.inertia/t2;
  u.translation_damping=u.mass/input.time_to_s;u.rotation_damping=u.inertia/input.time_to_s;
  u.length_floor=1e-15*input.length_to_m;u.shear_length_floor=1e-30*input.length_to_m;u.time=input.time_to_s;
  if(!Positive(t2)||!Positive(u.inertia)||!Positive(u.translation_stiffness)||!Positive(u.rotation_stiffness)||
     !Positive(u.translation_damping)||!Positive(u.rotation_damping)||!Positive(u.length_floor)||!Positive(u.shear_length_floor))return false;
  out=u;return true;
}
} // namespace detail

TL_TYPE25_HD inline bool ValidProperty(const Property& p) {
  if(!detail::Positive(p.mass_kg)||!detail::Positive(p.isotropic_inertia_kg_m2))return false;
  for(unsigned i=0;i<4;++i)
    if(!detail::Positive(p.stiffness[i])||!detail::Nonnegative(p.damping[i])||
       !detail::Positive(p.failure_positive[i])||!tl::math::fixed3::Finite(p.failure_negative[i])||p.failure_negative[i]>=0||
       !detail::Positive(p.failure_weight[i])||!detail::Positive(p.failure_exponent[i]))return false;
  return true;
}

// RMASS: each endpoint receives half of the Ileng=0 property mass and isotropic J.
struct MassCoefficients { double mass_kg=0,isotropic_inertia_kg_m2=0; };
TL_TYPE25_HD inline Status EndpointCoefficients(const Property& p,MassCoefficients& output) {
  if(!ValidProperty(p))return Status::InvalidInput;
  const double m=.5*p.mass_kg,j=.5*p.isotropic_inertia_kg_m2;
  if(!detail::Positive(m)||!detail::Positive(j))return Status::NonfiniteResult;
  output={m,j};return Status::Success;
}
} // namespace tl::fea::type25

#undef TL_TYPE25_HD
