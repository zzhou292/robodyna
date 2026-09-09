// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens; selected CNCOEF3B and starter material tables.
// Source pin/ranges and constant expression dependencies in owning manifest.
#pragma once
#include "QephGeometryWork.h"
#include "lib_src/materials/ShellElasticLaw1.h"

namespace tl::fea::qeph::detail {
// MYREAL8 constants retain source expression order; these are not raw REAL32
// literals. In particular ONEP414 is NOT sqrt(2) or a replacement decimal.
namespace force_constant {
constexpr double em20=1./1e20,em30=1./(1e20*1e10);
constexpr double one_over_12=1./12.,one_over_64=1./64.;
constexpr double four_over_3=4./3.,five_over_6=5./6.;
constexpr double zep01=1./100.,four_em3=4./1000.,five_em3=5./1000.;
constexpr double onep4=1.+4./10.;
constexpr double onep41=onep4+zep01;
constexpr double onep414=onep41+four_em3;
constexpr double zep06=1./10.-4./100.;
constexpr double threep464=3.+4./10.+zep06+four_em3;
constexpr double fivep333=16./3.;
constexpr double viscosity=zep01+five_em3;
}
struct MaterialWork {
  double thickness=0,thickness2=0,nu=0,g=0,young=0,a11=0,a12=0;
  double sound_speed=0,rho=0,volume=0,gs=0,dt=0,dm=0,dn=0;
  double g_sqrt=0,a11_sqrt=0,a12_sqrt=0,nu_sqrt=0,shf=0,shf_sqrt=0;
  tl::material::ShellElasticLaw1Coefficients elastic;
};
TL_QEPH_HD inline bool PrepareMaterial(const ReferenceInput& in,double area,
                                      double dt,MaterialWork& m) {
  using namespace force_constant;
  // hm_read_mat01 + selected CNCOEF3B: LAW1, IGTYP1, ITHK0, NPT0.
  m.thickness=in.thickness;
  m.thickness2=m.thickness*m.thickness;
  m.volume=m.thickness*area; m.dt=dt;
  m.rho=in.density; m.young=in.young_modulus; m.nu=in.poisson_ratio;
  if(!tl::material::PrepareShellElasticLaw1(m.young,m.nu,m.rho,m.elastic)) return false;
  m.g=m.elastic.g; m.a11=m.elastic.a11; m.a12=m.elastic.a12; m.sound_speed=m.elastic.sound_speed;
  m.g_sqrt=::sqrt(::fmax(0.,m.g)); m.a11_sqrt=::sqrt(::fmax(0.,m.a11));
  m.a12_sqrt=::sqrt(::fmax(0.,m.a12)); m.nu_sqrt=::sqrt(::fmax(0.,m.nu));
  m.shf=five_over_6; m.shf_sqrt=::sqrt(m.shf); m.gs=m.g*m.shf;
  m.dm=viscosity; m.dn=viscosity;
  const double values[]{m.thickness,m.thickness2,m.volume,m.dt,m.rho,m.young,m.nu,
      m.g,m.a11,m.a12,m.sound_speed,m.g_sqrt,m.a11_sqrt,m.a12_sqrt,m.nu_sqrt,
      m.shf,m.shf_sqrt,m.gs,m.dm,m.dn};
  for(double value:values) if(!tl::math::Finite(value)) return false;
  return Positive(m.thickness2)&&Positive(m.volume)&&Positive(m.g)&&Positive(m.a11)&&
      Positive(m.sound_speed)&&Positive(m.gs);
}
} // namespace tl::fea::qeph::detail
