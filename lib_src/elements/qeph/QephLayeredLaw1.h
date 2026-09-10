// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected OpenRadioss layered material composition, (C) 2026 Siemens.
#pragma once
#include "QephForce.h"
#include "lib_src/elements/sections/ShellLayeredLaw1.h"
#include "lib_src/elements/sections/ShellLayeredWork.h"

namespace tl::fea::qeph {
// Value wrappers only: native History/ForceTrial layouts and global LAW1 stay intact.
// Caller publishes the ordinary shell history and section sidecar together.
// The elastic sidecar has five stresses per point, with no plastic/rate history.
struct LayeredLaw1History { History shell{}; sections::ShellLayeredLaw1History section{}; };
struct LayeredLaw1ForceTrial {
  ForceTrial force{};
  sections::ShellLayeredLaw1History proposed_section{};
};
TL_QEPH_HD inline Status InitializeLayeredLaw1History(const ReferenceData& r,
    const tl::material::ShellElasticLaw1PointParameters& p,HistoryStamp stamp,LayeredLaw1History& output) noexcept {
  if(!sections::MatchesLayeredMaterial(p,r.input)) return Status::kInvalidInput;
  LayeredLaw1History candidate;
  const auto status=InitializeHistory(r,stamp,candidate.shell);
  if(status!=Status::kSuccess) return status;
  if(!tl::material::ValidShellElasticLaw1Point(p)) return Status::kInvalidInput;
  output=candidate; return Status::kSuccess;
}
// Native NPT3 LAW1 elastic stabilization; ITHICK1 updates physical thickness.
// Force projection and total stress/stabilization work retain the owning paths.
TL_QEPH_HD inline Status EvaluateLayeredLaw1Force(const ReferenceData& r,const tl::material::ShellElasticLaw1PointParameters& parameters,
    const LayeredLaw1History& accepted,
    const PrescribedInterval& interval,LayeredLaw1ForceTrial& output) noexcept {
  const auto& base=accepted.shell;
  if(!sections::MatchesLayeredMaterial(parameters,r.input)||
     !sections::MatchesLayeredSectionResultants(accepted.section,base.data())) return Status::kInvalidInput;
  if(!detail::SaneReference(r)||!base.matches_reference(r)) return Status::kInvalidReference;
  const auto& stamp=base.stamp();
  if(!detail::ValidHistoryValues(base.data())||!tl::math::Finite(stamp.time)||stamp.time<0||
     interval.base_time!=stamp.time||stamp.sample_index==UINT64_MAX||
     interval.sample_index!=stamp.sample_index+1) return Status::kInvalidInput;
  detail::GeometryWork geometry;
  const auto status=detail::PrepareGeometry(r,interval,geometry);
  if(status!=Status::kSuccess) return status;
  LayeredLaw1ForceTrial staged;
  auto& candidate=staged.force;
  candidate.kinematics=geometry.values; // Before native CNDT3 length mutation.
  detail::MaterialWork material;
  auto coefficients_input=r.input;
  coefficients_input.thickness=base.data().thickness; // Source ITHICK=1; native mass stays fixed.
  if(!detail::PrepareMaterial(coefficients_input,geometry.values.area,interval.dt,material))
    return Status::kNonfiniteResult;
  auto proposed=base.data();
  sections::ShellLayeredLaw1Input section_input;
  auto& dx=section_input.strain_curvature_increment;
  for(unsigned i=0;i<8;++i) dx[i]=geometry.values.regular_rate[i]*material.dt;
  const double xz=dx[3]; dx[3]=dx[4]; dx[4]=xz; // Native QEPH aliases: YZ,ZX.
  section_input.reference_thickness=material.thickness;
  section_input.reported_thickness=proposed.thickness;
  section_input.transverse_shear_modulus=material.gs;
  sections::ShellLayeredLaw1Result section;
  if(!sections::UpdateShellLayeredLaw1(parameters,accepted.section,section_input,section))
    return Status::kNonfiniteResult;
  const double dtinv=material.dt/::fmax(material.dt*material.dt,detail::force_constant::em20);
  const double viscosity=detail::force_constant::onep414*material.dm*material.rho*
      material.sound_speed*::sqrt(geometry.values.area)*dtinv;
  if(!sections::ApplyLayeredSectionWork(section,dx,material.thickness,geometry.values.area,viscosity,proposed))
    return Status::kNonfiniteResult;
  staged.proposed_section=section.history;
  detail::StiffnessDiagnostics(geometry,material,candidate.diagnostics);
  detail::LocalForceWork local;
  detail::ElasticForces(geometry,material,proposed,local);
  detail::StabilizationWork stabilization;
  detail::UpdateStabilization(geometry,material,proposed,stabilization);
  detail::StabilizationForces(geometry,material,proposed,stabilization,local);
  detail::ProjectForces(geometry,local,candidate.internal_force,candidate.internal_couple);
  auto& d=candidate.diagnostics;
  for(unsigned i=0;i<2;++i) d.internal_work_increment[i]=proposed.internal_work[i]-base.data().internal_work[i];
  d.hourglass_viscous_work_increment=proposed.hourglass_viscous_work-base.data().hourglass_viscous_work;
  if(!detail::ValidForceDiagnostics(d)||!detail::ValidHistoryValues(proposed)) return Status::kNonfiniteResult;
  for(unsigned n=0;n<4;++n)
    if(!detail::Finite(candidate.internal_force[n])||!detail::Finite(candidate.internal_couple[n]))
      return Status::kNonfiniteResult;
  const auto preparation=PreparePrescribedHistory(r,proposed,
      {interval.base_time+interval.dt,interval.sample_index},candidate.proposed_history);
  if(preparation!=Status::kSuccess) return Status::kNonfiniteResult;
  output=staged;
  return Status::kSuccess;
}
} // namespace tl::fea::qeph
