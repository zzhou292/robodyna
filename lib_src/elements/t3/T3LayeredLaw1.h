// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected OpenRadioss layered material composition, (C) 2026 Siemens.
#pragma once
#include "T3Force.h"
#include "lib_src/elements/sections/ShellLayeredLaw1.h"
#include "lib_src/elements/sections/ShellLayeredWork.h"

namespace tl::fea::t3 {
// Value wrappers only: native History/ForceTrial layouts and global LAW1 stay intact.
// Caller publishes the ordinary shell history and section sidecar together.
// The elastic sidecar has five stresses per point, with no plastic/rate history.
struct LayeredLaw1History { History shell{}; sections::ShellLayeredLaw1History section{}; };
struct LayeredLaw1ForceTrial {
  ForceTrial force{};
  sections::ShellLayeredLaw1History proposed_section{};
};
TL_T3_HD inline Status InitializeLayeredLaw1History(const ReferenceData& r,
    const tl::material::ShellElasticLaw1PointParameters& p,HistoryStamp stamp,LayeredLaw1History& output) noexcept {
  if(!sections::MatchesLayeredMaterial(p,r.input)) return Status::kInvalidInput;
  LayeredLaw1History candidate;
  const auto status=InitializeHistory(r,stamp,candidate.shell);
  if(status!=Status::kSuccess) return status;
  if(!tl::material::ValidShellElasticLaw1Point(p)) return Status::kInvalidInput;
  output=candidate; return Status::kSuccess;
}
// The native family equivalent-rate diagnostic is retained. LAW1 itself has
// no rate sensitivity or point filter history; physical thickness uses ITHICK1.
TL_T3_HD inline Status EvaluateLayeredLaw1Force(const ReferenceData& r,const tl::material::ShellElasticLaw1PointParameters& parameters,
    const LayeredLaw1History& accepted,
    const PrescribedInterval& in,LayeredLaw1ForceTrial& output) noexcept {
  const auto& base=accepted.shell;
  if(!sections::MatchesLayeredMaterial(parameters,r.input)||
     !sections::MatchesLayeredSectionResultants(accepted.section,base.data())) return Status::kInvalidInput;
  if(!detail::SaneReference(r)||!base.matches_reference(r)) return Status::kInvalidReference;
  const auto& stamp=base.stamp();
  if(!detail::ValidHistoryValues(base.data())||!tl::math::Finite(stamp.time)||stamp.time<0||
      stamp.time!=in.base_time||stamp.sample_index==UINT64_MAX||in.sample_index!=stamp.sample_index+1)
    return Status::kInvalidInput;
  double longest=0; auto status=detail::CheckPrescribed(r,in,longest);
  if(status!=Status::kSuccess) return status;
  detail::GeometryWork geometry;
  status=detail::CurrentGeometry(in.position,longest,geometry); if(status!=Status::kSuccess) return status;
  detail::MaterialWork material;
  auto coefficients_input=r.input;
  coefficients_input.thickness=base.data().thickness; // Source ITHICK=1; native mass stays fixed.
  if(!detail::PrepareMaterial(coefficients_input,geometry.kinematics.area,material)) return Status::kNonfiniteResult;
  status=detail::EvaluateRates(in,geometry); if(status!=Status::kSuccess) return status;
  auto& k=geometry.kinematics;
  k.base_time=in.base_time; k.position_time=in.base_time+in.dt; k.velocity_time=in.base_time+.5*in.dt;
  k.dt=in.dt; k.sample_index=in.sample_index; k.valid=true;
  LayeredLaw1ForceTrial staged;
  auto& candidate=staged.force; candidate.kinematics=k;
  auto proposed=base.data();
  sections::ShellLayeredLaw1Input section_input;
  auto& dx=section_input.strain_curvature_increment;
  const double rate_factor=in.dt/k.area;
  for(unsigned i=0;i<8;++i) dx[i]=k.raw_rate[i]*rate_factor; // Already native YZ,ZX.
  section_input.reference_thickness=material.thickness;
  section_input.reported_thickness=proposed.thickness;
  section_input.transverse_shear_modulus=material.gs;
  sections::ShellLayeredLaw1Result section;
  if(!sections::UpdateShellLayeredLaw1(parameters,accepted.section,section_input,section))
    return Status::kNonfiniteResult;
  using namespace detail::force_constant;
  const double dtinv=in.dt/::fmax(in.dt*in.dt,em20);
  const double eps_k2=(dx[5]*dx[5]+dx[6]*dx[6]+dx[5]*dx[6]+fourth*(dx[7]*dx[7]))*one_over_9*(proposed.thickness*proposed.thickness);
  const double eps_m2=four_over_3*(dx[0]*dx[0]+dx[1]*dx[1]+dx[0]*dx[1]+fourth*(dx[2]*dx[2]));
  proposed.equivalent_strain_rate=1.*(::sqrt(eps_k2+eps_m2)*dtinv)+(1.-1.)*proposed.equivalent_strain_rate;
  const double viscosity=onep414*material.dm*material.rho*material.elastic.sound_speed*::sqrt(k.area)*dtinv;
  if(!sections::ApplyLayeredSectionWork(section,dx,material.thickness,k.area,viscosity,proposed)) return Status::kNonfiniteResult;
  staged.proposed_section=section.history;
  detail::StiffnessDiagnostics(geometry,material,candidate.diagnostics);
  detail::LocalForceWork local;
  detail::InternalForces(geometry,material,proposed,local);
  detail::ProjectForces(geometry,local,candidate.internal_force,candidate.internal_couple);
  for(unsigned i=0;i<2;++i)
    candidate.diagnostics.internal_work_increment[i]=proposed.internal_work[i]-base.data().internal_work[i];
  if(!detail::ValidHistoryValues(proposed)||!detail::ValidForceDiagnostics(candidate.diagnostics)) return Status::kNonfiniteResult;
  for(unsigned n=0;n<3;++n)
    if(!detail::Finite(candidate.internal_force[n])||!detail::Finite(candidate.internal_couple[n])) return Status::kNonfiniteResult;
  if(PreparePrescribedHistory(r,proposed,{in.base_time+in.dt,in.sample_index},candidate.proposed_history)!=Status::kSuccess)
    return Status::kNonfiniteResult;
  output=staged; return Status::kSuccess;
}
} // namespace tl::fea::t3
