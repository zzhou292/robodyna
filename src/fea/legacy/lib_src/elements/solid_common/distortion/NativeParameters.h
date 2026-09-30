// SPDX-License-Identifier: AGPL-3.0-or-later
// SDISTOR_INI native-number preparation. The c031 SI-only leaf remains intact.
#pragma once
#include "ForceTypes.h"
#include "Parameters.h"
namespace tl::fea::solid_common::distortion::native {
// Native PM slots come from the original reader/update sequence in that working
// unit system. Do not rescale a previously evaluated SI FLD*velocity response.
TL_BRICK_HD inline Status PrepareParameters(double pm21,double pm22,double pm32,double pm100,
    double pm107,const double (&sig)[6],double rho,double speed,double volume,
    Parameters& output) noexcept {
  Parameters next;next.damping_coefficient=5.0/100.0;next.quadratic_limit=100;
  double fnu=1;
  if(pm21>static_cast<double>(.48999f))fnu=1.0/100.0;
  else if(pm21>static_cast<double>(.4f)){fnu=1-2*pm21;next.damping_coefficient=fnu*next.damping_coefficient;}
  else if(pm107>=180*pm32)fnu=10;
  const double c1=::fmax(pm32,pm100)+NativeOneP333*pm22,c2=fnu*c1;
  const double aj2=.5*(sig[0]*sig[0]+sig[1]*sig[1]+sig[2]*sig[2])+sig[3]*sig[3]+sig[4]*sig[4]+sig[5]*sig[5];
  const double es=::sqrt(3.0*aj2)/c1;
  if(!tl::math::Finite(c1)||!tl::math::Finite(c2)||!tl::math::Finite(aj2)||!tl::math::Finite(es))return Status::NonfiniteResult;
  double fes=::fmax(1.0/1000.0,100*es);fes=::fmin(1.0,fes);
  next.length=::pow(volume,1.0/3.0);
  const double caq=fes*next.damping_coefficient*rho*next.length;
  next.damping=.25*caq*speed*1.0;next.control_stiffness=c2*next.length*1.0;
  double minimum=sig[0];for(unsigned i=1;i<6;++i)minimum=::fmin(minimum,sig[i]);
  next.buckling_flag=c1<-minimum?1:0;
  if(!tl::math::Finite(next.length)||!tl::math::Finite(next.damping)||!tl::math::Finite(next.control_stiffness))return Status::NonfiniteResult;
  output=next;return Status::Success;
}
} // namespace tl::fea::solid_common::distortion::native
