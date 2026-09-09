// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// Pinned hm_read_mat01 PM elastic coefficients and centered SIGEPS01G branch.
// Provenance: lib_utest/qualification/qeph/source-manifest.json. Full AGPL
// notice and source pin are in lib_src/elements/qeph/LICENSE.md.
#pragma once
#include "lib_src/math/Quaternion.h" // Existing binary64 finite predicate.
#include <cmath>

#if defined(__CUDACC__)
#define TL_SHELL_LAW1_HD __host__ __device__
#else
#define TL_SHELL_LAW1_HD
#endif

namespace tl::material {
struct ShellElasticLaw1Coefficients {
  double g=0,a11=0,a12=0,sound_speed=0; // Pa,Pa,Pa,m/s.
};
struct ShellElasticLaw1Stress {
  double stress[5]{};         // XX,YY,XY,YZ,ZX, Pa.
  double bending_stress[3]{}; // XX,YY,XY; donor MOM stress-like units, Pa.
};

// Pure material coefficients; no section, state owner, history or clock.
// The native sound-speed density floor remains explicit; supported nu>=0.
TL_SHELL_LAW1_HD inline bool PrepareShellElasticLaw1(double young,double nu,double rho,
    ShellElasticLaw1Coefficients& output) noexcept {
  if(!tl::math::Finite(young)||young<=0||!tl::math::Finite(nu)||nu<0||nu>=.5||
     !tl::math::Finite(rho)||rho<=0) return false;
  ShellElasticLaw1Coefficients c;
  c.g=young/(2.*(1.+nu)); c.a11=young/(1.-nu*nu); c.a12=nu*c.a11;
  c.sound_speed=::sqrt(young/::fmax(rho,1./1e20));
  if(!tl::math::Finite(c.g)||c.g<=0||!tl::math::Finite(c.a11)||c.a11<=0||
     !tl::math::Finite(c.a12)||!tl::math::Finite(c.sound_speed)||c.sound_speed<=0) return false;
  output=c; return true;
}

// Exact centered SIGEPS01G linear accumulation. The formulation supplies GS
// and THK08; this helper neither chooses shear/thickness nor updates reported
// thickness, material strain, damping, work, stabilization or a sample index.
// dx order XX,YY,XY,YZ,ZX,KXX,KYY,KXY; first5 dimensionless,last3 inverse m.
// thk08 has m units. Preserve source addition order and staged failure bytes.
TL_SHELL_LAW1_HD inline bool UpdateShellElasticLaw1(
    const ShellElasticLaw1Coefficients& c,double gs,double thk08,
    const double (&dx)[8],const ShellElasticLaw1Stress& base,
    ShellElasticLaw1Stress& output) noexcept {
  if(!tl::math::Finite(c.g)||c.g<=0||!tl::math::Finite(c.a11)||c.a11<=0||
     !tl::math::Finite(c.a12)||c.a12<0||!tl::math::Finite(gs)||gs<=0||
     !tl::math::Finite(thk08)||thk08<=0) return false;
  for(double x:dx) if(!tl::math::Finite(x)) return false;
  for(double x:base.stress) if(!tl::math::Finite(x)) return false;
  for(double x:base.bending_stress) if(!tl::math::Finite(x)) return false;
  const double b1=c.a11*thk08,b2=c.a12*thk08,b3=c.g*thk08;
  ShellElasticLaw1Stress candidate;
  candidate.stress[0]=base.stress[0]+c.a11*dx[0]+c.a12*dx[1];
  candidate.stress[1]=base.stress[1]+c.a12*dx[0]+c.a11*dx[1];
  candidate.stress[2]=base.stress[2]+c.g*dx[2];
  candidate.stress[3]=base.stress[3]+gs*dx[3];
  candidate.stress[4]=base.stress[4]+gs*dx[4];
  candidate.bending_stress[0]=base.bending_stress[0]+b1*dx[5]+b2*dx[6];
  candidate.bending_stress[1]=base.bending_stress[1]+b1*dx[6]+b2*dx[5];
  candidate.bending_stress[2]=base.bending_stress[2]+b3*dx[7];
  for(double x:candidate.stress) if(!tl::math::Finite(x)) return false;
  for(double x:candidate.bending_stress) if(!tl::math::Finite(x)) return false;
  output=candidate; return true;
}
} // namespace tl::material
#undef TL_SHELL_LAW1_HD
