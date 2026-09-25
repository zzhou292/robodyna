// SPDX-License-Identifier: AGPL-3.0-or-later
// Coherent selected C3FORC3, OpenRadioss (C) 2026 Siemens; see FORCE.md map.
#pragma once
#include "../ShellGlobalLaw1Profile.h"
#include "T3History.h"
#include "T3Law1.h"
#include "T3ForceProjection.h"
#include "T3StiffnessDiagnostics.h"
namespace tl::fea::t3 {
// Pure prescribed endpoint/midpoint value operation. Positive internal loads
// are subtracted from nodal RHS. No owner/clock/native runtime/h=0 force cache.
// All inputs and caller output survive failure, even when base aliases output.
namespace detail {
TL_T3_HD inline Status EvaluateForceWithThickness(const ReferenceData& r,const History& base,
    const PrescribedInterval& in,double coefficient_thickness,ForceTrial& output) noexcept {
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
  auto coefficient_input=r.input;
  coefficient_input.thickness=coefficient_thickness;
  if(!detail::PrepareMaterial(coefficient_input,geometry.kinematics.area,material)) return Status::kNonfiniteResult;
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
} // namespace detail

// Legacy fixed-coefficient-thickness API and ordinary shell history are unchanged.
TL_T3_HD inline Status EvaluateForce(const ReferenceData& r,const History& base,
    const PrescribedInterval& interval,ForceTrial& output) noexcept {
  return detail::EvaluateForceWithThickness(r,base,interval,r.input.thickness,output);
}

// Explicit native NPT0 analytic LAW1. No Gauss-point history is created.
// The physical owner must retain the resolved profile immutably across calls.
// C3COEF3 uses accepted thickness directly; it has no CNCOEF3B floor.
TL_T3_HD inline Status EvaluateGlobalLaw1Force(const ShellGlobalLaw1Profile& profile,
    const ReferenceData& r,const History& base,const PrescribedInterval& interval,
    ForceTrial& output) noexcept {
  if(!shell_global_law1::Valid(profile)) return Status::kInvalidInput;
  const double thickness=profile.thickness==ShellLaw1Thickness::Reference?
      r.input.thickness : base.data().thickness;
  return detail::EvaluateForceWithThickness(r,base,interval,thickness,output);
}
} // namespace tl::fea::t3
