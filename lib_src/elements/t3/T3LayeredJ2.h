// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected OpenRadioss layered material composition, (C) 2026 Siemens.
#pragma once
#include "T3Force.h"
#include "lib_src/elements/sections/ShellLayeredJ2Work.h"

namespace tl::fea::t3 {
// Value wrappers only: native History/ForceTrial layouts and LAW1 stay intact.
// Caller publishes the ordinary shell history and section sidecar together.
// The immutable prepared material/curve is a caller binding, not owned here.
struct LayeredJ2History { History shell{}; sections::ShellLayeredJ2History section{}; };
struct LayeredJ2ForceTrial {
  ForceTrial force{};
  sections::ShellLayeredJ2History proposed_section{};
  sections::ShellLayeredJ2Diagnostics section_diagnostics{};
};
TL_T3_HD inline Status InitializeLayeredJ2History(const ReferenceData& r,
    const sections::PointParameters& p,HistoryStamp stamp,LayeredJ2History& output) noexcept {
  if(!sections::MatchesLayeredJ2Material(p,r.input)) return Status::kInvalidInput;
  LayeredJ2History candidate;
  const auto status=InitializeHistory(r,stamp,candidate.shell);
  if(status!=Status::kSuccess) return status;
  sections::ShellLayeredJ2Input input;
  input.reference_thickness=input.reported_thickness=r.input.thickness;
  input.transverse_shear_modulus=p.shear_modulus*(5./6.);
  sections::ShellLayeredJ2Result checked;
  if(sections::UpdateShellLayeredJ2(p,candidate.section,input,checked)!=sections::PointStatus::Ok)
    return Status::kInvalidInput;
  output=candidate; return Status::kSuccess;
}
// Equivalent strain rate remains diagnostic; the point law is rate independent.
TL_T3_HD inline Status EvaluateLayeredJ2Force(const ReferenceData& r,const sections::PointParameters& parameters,
    const LayeredJ2History& accepted,
    const PrescribedInterval& in,LayeredJ2ForceTrial& output) noexcept {
  const auto& base=accepted.shell;
  if(!sections::MatchesLayeredJ2Material(parameters,r.input)||
     !sections::MatchesLayeredJ2Resultants(accepted.section,base.data())) return Status::kInvalidInput;
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
  if(!detail::PrepareMaterial(r.input,geometry.kinematics.area,material)) return Status::kNonfiniteResult;
  status=detail::EvaluateRates(in,geometry); if(status!=Status::kSuccess) return status;
  auto& k=geometry.kinematics;
  k.base_time=in.base_time; k.position_time=in.base_time+in.dt; k.velocity_time=in.base_time+.5*in.dt;
  k.dt=in.dt; k.sample_index=in.sample_index; k.valid=true;
  LayeredJ2ForceTrial staged;
  auto& candidate=staged.force; candidate.kinematics=k;
  auto proposed=base.data();
  sections::ShellLayeredJ2Input section_input;
  auto& dx=section_input.strain_curvature_increment;
  const double rate_factor=in.dt/k.area;
  for(unsigned i=0;i<8;++i) dx[i]=k.raw_rate[i]*rate_factor; // Already native YZ,ZX.
  section_input.reference_thickness=material.thickness;
  section_input.reported_thickness=proposed.thickness;
  section_input.transverse_shear_modulus=material.gs;
  sections::ShellLayeredJ2Result section;
  if(sections::UpdateShellLayeredJ2(parameters,accepted.section,section_input,section)!=sections::PointStatus::Ok)
    return Status::kNonfiniteResult;
  using namespace detail::force_constant;
  const double dtinv=in.dt/::fmax(in.dt*in.dt,em20);
  const double eps_k2=(dx[5]*dx[5]+dx[6]*dx[6]+dx[5]*dx[6]+fourth*(dx[7]*dx[7]))*one_over_9*(proposed.thickness*proposed.thickness);
  const double eps_m2=four_over_3*(dx[0]*dx[0]+dx[1]*dx[1]+dx[0]*dx[1]+fourth*(dx[2]*dx[2]));
  proposed.equivalent_strain_rate=1.*(::sqrt(eps_k2+eps_m2)*dtinv)+(1.-1.)*proposed.equivalent_strain_rate;
  const double viscosity=onep414*material.dm*material.rho*material.elastic.sound_speed*::sqrt(k.area)*dtinv;
  if(!sections::ApplyLayeredJ2Work(section,dx,material.thickness,k.area,viscosity,proposed)) return Status::kNonfiniteResult;
  staged.proposed_section=section.history; staged.section_diagnostics=section.diagnostics;
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
