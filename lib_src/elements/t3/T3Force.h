// SPDX-License-Identifier: AGPL-3.0-or-later
// Coherent selected C3FORC3, OpenRadioss (C) 2026 Siemens; see FORCE.md map.
#pragma once
#include "T3History.h"
#include "T3Law1.h"
#include "T3ForceProjection.h"
#include "T3StiffnessDiagnostics.h"
namespace tl::fea::t3 {
// Pure prescribed endpoint/midpoint value operation. Positive internal loads
// are subtracted from nodal RHS. No owner/clock/native runtime/h=0 force cache.
// All inputs and caller output survive failure, even when base aliases output.
TL_T3_HD inline Status EvaluateForce(const ReferenceData& r,const History& base,
    const PrescribedInterval& in,ForceTrial& output) noexcept {
  if(r.input.placement!=ShellReferencePlacement::Centered) return Status::kInvalidInput;
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
  ForceTrial candidate; candidate.kinematics=k;
  auto proposed=base.data();
  if(!detail::UpdateLaw1(geometry,material,in.dt,proposed)) return Status::kNonfiniteResult;
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
  output=candidate; return Status::kSuccess;
}
} // namespace tl::fea::t3
