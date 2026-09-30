// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected C3COEF3/PM expressions, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3CurrentGeometry.h"
#include "T3ForceConstants.h"
#include "lib_src/materials/ShellElasticLaw1.h"
#include "lib_src/elements/ShellPlacementCoefficients.h"
namespace tl::fea::t3::detail {
struct MaterialWork {
  tl::material::ShellElasticLaw1Coefficients elastic;
  double thickness=0,thickness2=0,volume=0,nu=0,rho=0,gs=0,shf=0,offset=0,dm=0;
};
TL_T3_HD inline bool PrepareMaterial(const ReferenceInput& in,double area,MaterialWork& output,
    ShellReferencePlacement placement=ShellReferencePlacement::Centered) {
  using namespace force_constant;
  MaterialWork m;
  ShellPlacementCoefficients positioned;
  if(!PrepareShellPlacementCoefficients(placement,in.thickness,positioned)) return false;
  if(!tl::material::PrepareShellElasticLaw1(in.young_modulus,in.poisson_ratio,in.density,m.elastic)) return false;
  m.thickness=in.thickness; m.thickness2=m.thickness*m.thickness;
  m.volume=m.thickness*area; m.nu=in.poisson_ratio; m.rho=in.density;
  // Preserve selected C3COEF3's ISH0 expression, including the denominator.
  const double fac1=2.*(1.+m.nu)*m.thickness2,fsh=five_over_6,ish=0.;
  const double denominator=fsh*area+fac1;
  if(!Positive(m.thickness2)||!Positive(m.volume)||!Positive(fac1)||!Positive(denominator)) return false;
  m.shf=fsh*(1.-ish+ish*fac1/denominator);
  m.gs=m.elastic.g*m.shf; m.offset=positioned.offset; m.dm=viscosity;
  if(!Positive(m.shf)||!Positive(m.gs)||!tl::math::Finite(m.offset)) return false;
  output=m; return true;
}
} // namespace tl::fea::t3::detail
