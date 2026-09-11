// SPDX-License-Identifier: AGPL-3.0-or-later
// One native layered force path per family; section policy changes only its
// material caller and explicit OFF admission. Existing force order is retained.
#pragma once
#include "QephForce.h"
#include "QephPlasticStabilization.h"
#include "lib_src/elements/sections/ShellLayeredJ2ForceAdapter.h"

namespace tl::fea::qeph::detail {
template<class Accepted,class Trial,class Adapter>
TL_QEPH_HD inline Status EvaluateLayeredForce(const ReferenceData& r,const sections::PointParameters& parameters,
    const Accepted& accepted,const PrescribedInterval& interval,Trial& output,const Adapter& adapter) noexcept {
  const auto& base=accepted.shell;
  if(!sections::ValidLayeredJ2Parameters(parameters)||
     !sections::MatchesLayeredJ2Material(parameters,r.input)||
     !adapter.Matches(base.data(),r.input.placement)) return Status::kInvalidInput;
  if(!SaneReference(r)||!base.matches_reference(r)) return Status::kInvalidReference;
  const auto& stamp=base.stamp();
  if(!ValidHistoryValues(base.data(),Adapter::admits_failure)||!tl::math::Finite(stamp.time)||stamp.time<0||
     interval.base_time!=stamp.time||stamp.sample_index==UINT64_MAX||
     interval.sample_index!=stamp.sample_index+1) return Status::kInvalidInput;
  GeometryWork geometry;
  const auto status=PrepareGeometry(r,interval,geometry);
  if(status!=Status::kSuccess) return status;
  Trial staged;
  auto& candidate=staged.force;
  candidate.kinematics=geometry.values; // Before native CNDT3 length mutation.
  MaterialWork material;
  auto coefficients_input=r.input;
  coefficients_input.thickness=base.data().thickness; // Source ITHICK=1; native mass stays fixed.
  if(!PrepareMaterial(coefficients_input,geometry.values.area,interval.dt,material,r.input.placement))
    return Status::kNonfiniteResult;
  auto proposed=base.data();
  sections::ShellLayeredJ2Input section_input;
  section_input.placement=r.input.placement;
  auto& dx=section_input.strain_curvature_increment;
  for(unsigned i=0;i<8;++i) dx[i]=geometry.values.regular_rate[i]*material.dt;
  const double xz=dx[3]; dx[3]=dx[4]; dx[4]=xz; // Native QEPH aliases: YZ,ZX.
  section_input.reference_thickness=material.thickness;
  section_input.reported_thickness=proposed.thickness;
  section_input.transverse_shear_modulus=material.gs;
  section_input.dt=material.dt;
  typename Adapter::Result section;
  if(adapter.Update(section_input,interval.base_time+interval.dt,section)!=sections::PointStatus::Ok)
    return Status::kNonfiniteResult;
  // SIGEPS44C replaces the startup SSP before MULAWC viscosity and CNDT3.
  // This is the validated LAW44 coefficient, including for an inactive parent.
  material.sound_speed=parameters.sound_speed;
  const double dtinv=material.dt/::fmax(material.dt*material.dt,force_constant::em20);
  const double viscosity=force_constant::onep414*material.dm*material.rho*
      material.sound_speed*::sqrt(geometry.values.area)*dtinv;
  if(!adapter.Apply(section,dx,material.thickness,geometry.values.area,viscosity,proposed,r.input.placement))
    return Status::kNonfiniteResult;
  Adapter::Publish(section,staged);
  StiffnessDiagnostics(geometry,material,candidate.diagnostics,proposed.active);
  LocalForceWork local;
  ElasticForces(geometry,material,proposed,local);
  StabilizationWork stabilization;
  UpdateStabilization(geometry,material,proposed,stabilization,proposed.active);
  if(!CorrectPlasticStabilization(material,Adapter::Diagnostics(section).mean_tangent_ratio,
      Adapter::Diagnostics(section).minimum_tangent_ratio,Adapter::Diagnostics(section).last_point_yield_before_pa,
      proposed,stabilization)) return Status::kNonfiniteResult;
  StabilizationForces(geometry,material,proposed,stabilization,local,proposed.active);
  ProjectForces(geometry,local,candidate.internal_force,candidate.internal_couple);
  auto& d=candidate.diagnostics;
  for(unsigned i=0;i<2;++i) d.internal_work_increment[i]=proposed.internal_work[i]-base.data().internal_work[i];
  d.hourglass_viscous_work_increment=proposed.hourglass_viscous_work-base.data().hourglass_viscous_work;
  if(!ValidForceDiagnostics(d,proposed.active)||!ValidHistoryValues(proposed,Adapter::admits_failure)) return Status::kNonfiniteResult;
  for(unsigned n=0;n<4;++n)
    if(!Finite(candidate.internal_force[n])||!Finite(candidate.internal_couple[n]))
      return Status::kNonfiniteResult;
  const auto preparation=Adapter::admits_failure?
      PrepareFailurePrescribedHistory(r,proposed,{interval.base_time+interval.dt,interval.sample_index},candidate.proposed_history):
      PreparePrescribedHistory(r,proposed,{interval.base_time+interval.dt,interval.sample_index},candidate.proposed_history);
  if(preparation!=Status::kSuccess) return Status::kNonfiniteResult;
  output=staged;
  return Status::kSuccess;
}
} // namespace tl::fea::qeph::detail
