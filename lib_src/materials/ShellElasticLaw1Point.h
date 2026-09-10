// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected native SIGEPS01C ISMSTR2, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "ShellElasticLaw1.h"
#if defined(__CUDACC__)
#define TL_LAW1_POINT_HD __host__ __device__
#else
#define TL_LAW1_POINT_HD
#endif
namespace tl::material {
struct ShellElasticLaw1PointParameters {
  double young_pa=0,poisson_ratio=0,density_kg_m3=0;
  ShellElasticLaw1Coefficients elastic;
};
// Elastic histories have no invented plastic strain, rate filter or yield data.
struct ShellElasticLaw1PointHistory { double stress[5]{}; };
struct ShellElasticLaw1PointInput {
  double strain_increment[5]{}; // XX,YY,engineering XY,YZ,ZX.
  double transverse_shear_modulus=0,layer_thickness=0,reported_thickness=0;
};
struct ShellElasticLaw1PointResult {
  ShellElasticLaw1PointHistory history;
  double elastic_thickness_strain=0,reported_thickness=0;
};
TL_LAW1_POINT_HD inline bool PrepareShellElasticLaw1Point(double young,double nu,double rho,
    ShellElasticLaw1PointParameters& output) noexcept {
  ShellElasticLaw1PointParameters next;
  if(!PrepareShellElasticLaw1(young,nu,rho,next.elastic)) return false;
  next.young_pa=young;next.poisson_ratio=nu;next.density_kg_m3=rho;
  output=next;return true;
}
TL_LAW1_POINT_HD inline bool ValidShellElasticLaw1Point(const ShellElasticLaw1PointParameters& p) noexcept {
  ShellElasticLaw1Coefficients c;
  return PrepareShellElasticLaw1(p.young_pa,p.poisson_ratio,p.density_kg_m3,c)&&
    p.elastic.g==c.g&&p.elastic.a11==c.a11&&p.elastic.a12==c.a12&&p.elastic.sound_speed==c.sound_speed;
}
// One native point update at OFF=1, ISMSTR2. The caller supplies the actual
// native GS, THKLY and running physical THK. No clock, integration or allocation.
// All inputs are consumed before publication; every failure preserves output.
TL_LAW1_POINT_HD inline bool UpdateShellElasticLaw1Point(const ShellElasticLaw1PointParameters& p,
    const ShellElasticLaw1PointHistory& base,const ShellElasticLaw1PointInput& in,
    ShellElasticLaw1PointResult& output) noexcept {
  if(!ValidShellElasticLaw1Point(p)||!tl::math::Finite(in.transverse_shear_modulus)||
     !(in.transverse_shear_modulus>0)||!tl::math::Finite(in.layer_thickness)||!(in.layer_thickness>0)||
     !tl::math::Finite(in.reported_thickness)||!(in.reported_thickness>=1.e-30)) return false;
  for(double x:in.strain_increment) if(!tl::math::Finite(x)) return false;
  for(double x:base.stress) if(!tl::math::Finite(x)) return false;
  const auto& d=in.strain_increment;const auto& c=p.elastic;
  ShellElasticLaw1PointResult next;
  next.history.stress[0]=base.stress[0]+c.a11*d[0]+c.a12*d[1];
  next.history.stress[1]=base.stress[1]+c.a12*d[0]+c.a11*d[1];
  next.history.stress[2]=base.stress[2]+c.g*d[2];
  next.history.stress[3]=base.stress[3]+in.transverse_shear_modulus*d[3];
  next.history.stress[4]=base.stress[4]+in.transverse_shear_modulus*d[4];
  next.elastic_thickness_strain=-p.poisson_ratio*(d[0]+d[1])/(1.-p.poisson_ratio);
  next.reported_thickness=in.reported_thickness+next.elastic_thickness_strain*in.layer_thickness*1.;
  for(double x:next.history.stress) if(!tl::math::Finite(x)) return false;
  if(!tl::math::Finite(next.elastic_thickness_strain)||!tl::math::Finite(next.reported_thickness)||
     !(next.reported_thickness>=1.e-30)) return false;
  output=next;return true;
}
} // namespace tl::material
#undef TL_LAW1_POINT_HD
