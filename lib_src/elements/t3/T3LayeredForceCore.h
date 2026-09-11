// SPDX-License-Identifier: AGPL-3.0-or-later
// One native layered force path per family; section policy changes only its
// material caller and explicit OFF admission. Existing force order is retained.
#pragma once
#include "T3Force.h"
#include "lib_src/elements/sections/ShellLayeredJ2ForceAdapter.h"

namespace tl::fea::t3::detail {
template<class Accepted,class Trial,class Adapter>
TL_T3_HD inline Status EvaluateLayeredForce(const ReferenceData& r,const sections::PointParameters& parameters,
    const Accepted& accepted,const PrescribedInterval& in,Trial& output,const Adapter& adapter) noexcept {
  const auto& base=accepted.shell;
  if(!sections::MatchesLayeredJ2Material(parameters,r.input)||
     !adapter.Matches(base.data(),r.input.placement)) return Status::kInvalidInput;
  if(!SaneReference(r)||!base.matches_reference(r)) return Status::kInvalidReference;
  const auto& stamp=base.stamp();
  if(!ValidHistoryValues(base.data(),Adapter::admits_failure)||!tl::math::Finite(stamp.time)||stamp.time<0||
      stamp.time!=in.base_time||stamp.sample_index==UINT64_MAX||in.sample_index!=stamp.sample_index+1)
    return Status::kInvalidInput;
  double longest=0; auto status=CheckPrescribed(r,in,longest);
  if(status!=Status::kSuccess) return status;
  GeometryWork geometry;
  status=CurrentGeometry(in.position,longest,geometry); if(status!=Status::kSuccess) return status;
  MaterialWork material;
  auto coefficients_input=r.input;
  coefficients_input.thickness=base.data().thickness; // Source ITHICK=1; native mass stays fixed.
  if(!PrepareMaterial(coefficients_input,geometry.kinematics.area,material,r.input.placement)) return Status::kNonfiniteResult;
  status=EvaluateRates(in,geometry); if(status!=Status::kSuccess) return status;
  auto& k=geometry.kinematics;
  k.base_time=in.base_time; k.position_time=in.base_time+in.dt; k.velocity_time=in.base_time+.5*in.dt;
  k.dt=in.dt; k.sample_index=in.sample_index; k.valid=true;
  Trial staged;
  auto& candidate=staged.force; candidate.kinematics=k;
  auto proposed=base.data();
  sections::ShellLayeredJ2Input section_input;
  section_input.placement=r.input.placement;
  auto& dx=section_input.strain_curvature_increment;
  const double rate_factor=in.dt/k.area;
  for(unsigned i=0;i<8;++i) dx[i]=k.raw_rate[i]*rate_factor; // Already native YZ,ZX.
  section_input.reference_thickness=material.thickness;
  section_input.reported_thickness=proposed.thickness;
  section_input.transverse_shear_modulus=material.gs;
  section_input.dt=in.dt;
  typename Adapter::Result section;
  if(adapter.Update(section_input,in.base_time+in.dt,section)!=sections::PointStatus::Ok)
    return Status::kNonfiniteResult;
  using namespace force_constant;
  const double dtinv=in.dt/::fmax(in.dt*in.dt,em20);
  const double eps_k2=(dx[5]*dx[5]+dx[6]*dx[6]+dx[5]*dx[6]+fourth*(dx[7]*dx[7]))*one_over_9*(proposed.thickness*proposed.thickness);
  const double eps_m2=four_over_3*(dx[0]*dx[0]+dx[1]*dx[1]+dx[0]*dx[1]+fourth*(dx[2]*dx[2]));
  proposed.equivalent_strain_rate=1.*(::sqrt(eps_k2+eps_m2)*dtinv)+(1.-1.)*proposed.equivalent_strain_rate;
  const double viscosity=onep414*material.dm*material.rho*material.elastic.sound_speed*::sqrt(k.area)*dtinv;
  if(!adapter.Apply(section,dx,material.thickness,k.area,viscosity,proposed,r.input.placement)) return Status::kNonfiniteResult;
  Adapter::Publish(section,staged);
  StiffnessDiagnostics(geometry,material,candidate.diagnostics,proposed.active);
  LocalForceWork local;
  InternalForces(geometry,material,proposed,local);
  ProjectForces(geometry,local,candidate.internal_force,candidate.internal_couple);
  for(unsigned i=0;i<2;++i)
    candidate.diagnostics.internal_work_increment[i]=proposed.internal_work[i]-base.data().internal_work[i];
  if(!ValidHistoryValues(proposed,Adapter::admits_failure)||!ValidForceDiagnostics(candidate.diagnostics,proposed.active)) return Status::kNonfiniteResult;
  for(unsigned n=0;n<3;++n)
    if(!Finite(candidate.internal_force[n])||!Finite(candidate.internal_couple[n])) return Status::kNonfiniteResult;
  const auto preparation=Adapter::admits_failure?
      PrepareFailurePrescribedHistory(r,proposed,{in.base_time+in.dt,in.sample_index},candidate.proposed_history):
      PreparePrescribedHistory(r,proposed,{in.base_time+in.dt,in.sample_index},candidate.proposed_history);
  if(preparation!=Status::kSuccess) return Status::kNonfiniteResult;
  output=staged; return Status::kSuccess;
}
} // namespace tl::fea::t3::detail
