// SPDX-License-Identifier: AGPL-3.0-or-later
// Adapted from OpenRadioss (C) 2026 Siemens; see qualification/shell_layered_j2/README.md.
#pragma once
#include "lib_src/materials/TabulatedShellPlasticity.h"

#if defined(__CUDACC__)
#define TL_SHELL_SECTION_HD __host__ __device__
#else
#define TL_SHELL_SECTION_HD
#endif

namespace tl::fea::sections {
using PointParameters=tl::material::TabulatedShellPlasticityParameters;
using PointStatus=tl::material::TabulatedShellPlasticityStatus;
// Ordinary centered NIP=3 shell section. The donor uses separate force and
// moment tables: WM is deliberately NOT WF*Z or an exact 1/12.
TL_SHELL_SECTION_HD inline double LayerPosition(unsigned i) noexcept { return .5*(static_cast<double>(i)-1.); }
TL_SHELL_SECTION_HD inline double LayerForceWeight(unsigned i) noexcept { return i==1?.5:.25; }
TL_SHELL_SECTION_HD inline double LayerMomentWeight(unsigned i) noexcept {
  return (static_cast<double>(i)-1.)*static_cast<double>(0.0833333f);
}
struct ShellLayeredJ2History { tl::material::TabulatedShellPlasticityHistory point[3]{}; };
struct ShellLayeredJ2Input {
  double strain_curvature_increment[8]{}; // XX,YY,engineering XY,YZ,ZX,KXX,KYY,KXY.
  // reference_thickness is the interval's effective force thickness (native THK0).
  // The source ITHICK=1 adapters supply accepted reported thickness here.
  double reference_thickness=0, reported_thickness=0, transverse_shear_modulus=0;
  double dt=0; // Native interval duration, required by the optional rate branch.
};
struct ShellLayeredJ2Diagnostics {
  double plastic_work_density_increment=0; // WF-weighted native point diagnostic, J/m3.
  double maximum_plastic_strain=0, mean_plastic_strain=0;
  double minimum_tangent_ratio=1;
  double mean_tangent_ratio=1, mean_yield_before_pa=0, last_point_yield_before_pa=0;
};

// Native CZFORC3/C3FORC3 EPSD_PG, before CMAIN3 changes reported thickness.
// The scalar is common to all thickness points; their filtered histories are not.
TL_SHELL_SECTION_HD inline double LayeredJ2TotalStrainRate(const ShellLayeredJ2Input& in) noexcept {
  const auto& d=in.strain_curvature_increment;
  const double dtinv=in.dt/::fmax(in.dt*in.dt,1.e-20);
  const double bending=(d[5]*d[5]+d[6]*d[6]+d[5]*d[6]+.25*(d[7]*d[7]))*
      (1./9.)*(in.reported_thickness*in.reported_thickness);
  const double membrane=(4./3.)*(d[0]*d[0]+d[1]*d[1]+d[0]*d[1]+.25*(d[2]*d[2]));
  return ::sqrt(bending+membrane)*dtinv;
}
struct ShellLayeredJ2Result {
  ShellLayeredJ2History history{};
  double material_stress[5]{}, bending_stress[3]{}; // FOR and MOM, both Pa.
  double reported_thickness=0;
  ShellLayeredJ2Diagnostics diagnostics{};
};

// FOR*t and MOM*t^2 are the physical force/moment resultants. Keeping the same
// ordered accumulation here and at admission binds sidecar stress to the shell.
TL_SHELL_SECTION_HD inline void LayeredJ2Resultants(const ShellLayeredJ2History& h,
    double (&force)[5],double (&moment)[3]) noexcept {
  for(double& x:force) x=0;
  for(double& x:moment) x=0;
  for(unsigned p=0;p<3;++p) {
    for(unsigned c=0;c<5;++c) force[c]=force[c]+LayerForceWeight(p)*h.point[p].stress[c];
    for(unsigned c=0;c<3;++c) moment[c]=moment[c]+LayerMomentWeight(p)*h.point[p].stress[c];
  }
}

// The effective force thickness stays fixed within this interval. The point
// law supplies two native thickness additions for each layer. Failure is atomic
// even if the third layer rejects its curve domain after the first two succeed.
TL_SHELL_SECTION_HD inline PointStatus UpdateShellLayeredJ2(const PointParameters& p,
    const ShellLayeredJ2History& accepted,const ShellLayeredJ2Input& in,
    ShellLayeredJ2Result& output) noexcept {
  if(!tl::math::Finite(in.reference_thickness)||!(in.reference_thickness>0)||
     !tl::math::Finite(in.reported_thickness)||!(in.reported_thickness>=1.e-30)||
     !tl::math::Finite(in.transverse_shear_modulus)||!(in.transverse_shear_modulus>0))
    return PointStatus::InvalidIncrement;
  for(double x:in.strain_curvature_increment)
    if(!tl::math::Finite(x)) return PointStatus::InvalidIncrement;
  double total_rate=0;
  if(p.rate.enabled) {
    if(!tl::math::Finite(in.dt)||!(in.dt>0)) return PointStatus::InvalidIncrement;
    total_rate=LayeredJ2TotalStrainRate(in);
    if(!tl::math::Finite(total_rate)) return PointStatus::NonfiniteResult;
  }
  ShellLayeredJ2Result candidate; candidate.reported_thickness=in.reported_thickness;
  candidate.diagnostics.mean_tangent_ratio=0; // Ordered sum; startup defaults to elastic unity.
  for(unsigned layer=0;layer<3;++layer) {
    tl::material::TabulatedShellPlasticityInput point_input;
    point_input.transverse_shear_modulus=in.transverse_shear_modulus;
    point_input.dt=in.dt; point_input.total_strain_rate_per_s=total_rate;
    const double z=LayerPosition(layer)*in.reference_thickness;
    for(unsigned c=0;c<3;++c)
      point_input.strain_increment[c]=in.strain_curvature_increment[c]+z*in.strain_curvature_increment[c+5];
    for(unsigned c=3;c<5;++c) point_input.strain_increment[c]=in.strain_curvature_increment[c];
    tl::material::TabulatedShellPlasticityResult point;
    const auto status=tl::material::UpdateLaw44ShellPlasticity(p,accepted.point[layer],point_input,point);
    if(status!=PointStatus::Ok) return status;
    candidate.history.point[layer]=point.history;
    const double weight=LayerForceWeight(layer),layer_thickness=weight*in.reference_thickness;
    candidate.reported_thickness=candidate.reported_thickness+point.elastic_thickness_strain*layer_thickness;
    candidate.reported_thickness=candidate.reported_thickness+point.plastic_thickness_strain*layer_thickness;
    auto& d=candidate.diagnostics;
    d.plastic_work_density_increment=d.plastic_work_density_increment+weight*point.plastic_work_density;
    d.mean_plastic_strain=d.mean_plastic_strain+weight*point.history.plastic_strain;
    d.maximum_plastic_strain=::fmax(d.maximum_plastic_strain,point.history.plastic_strain);
    d.minimum_tangent_ratio=::fmin(d.minimum_tangent_ratio,point.tangent_ratio);
    d.mean_tangent_ratio=d.mean_tangent_ratio+weight*point.tangent_ratio;
    d.mean_yield_before_pa=d.mean_yield_before_pa+weight*point.yield_before_pa;
    d.last_point_yield_before_pa=point.yield_before_pa;
  }
  LayeredJ2Resultants(candidate.history,candidate.material_stress,candidate.bending_stress);
  if(!tl::math::Finite(candidate.reported_thickness)||!(candidate.reported_thickness>=1.e-30)||
     !tl::math::Finite(candidate.diagnostics.plastic_work_density_increment)||
     !tl::math::Finite(candidate.diagnostics.mean_plastic_strain)||
     !tl::math::Finite(candidate.diagnostics.mean_tangent_ratio)||
     !tl::math::Finite(candidate.diagnostics.mean_yield_before_pa)||
     !tl::math::Finite(candidate.diagnostics.last_point_yield_before_pa)) return PointStatus::NonfiniteResult;
  for(double x:candidate.material_stress) if(!tl::math::Finite(x)) return PointStatus::NonfiniteResult;
  for(double x:candidate.bending_stress) if(!tl::math::Finite(x)) return PointStatus::NonfiniteResult;
  output=candidate; return PointStatus::Ok;
}
} // namespace tl::fea::sections
