// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss (C) 2026 Siemens; coherent selected QEPH LAW1 prescribed force.
#pragma once
#include "../ShellGlobalLaw1Profile.h"
#include "QephHistory.h"
#include "QephLaw1.h"
#include "QephStabilizationForces.h"
#include "QephForceProjectionUnits.h"
#include "QephStiffnessDiagnostics.h"

namespace tl::fea::qeph {
// Native positive internal forces/couples: a nodal RHS subtracts these values.
// No Fortran runtime, owner, force cache, clock or dt0 initial-force operation.
// Centered global LAW1, CVIS1, DM=DN=.015, ISROT0/IDRIL0. Exact base
// reference/time and next sample identity; every output/base byte survives
// failure, including late reported-thickness or force/diagnostic arithmetic.
namespace detail {
TL_QEPH_HD inline Status EvaluateForceWithThickness(const ReferenceData& r,const History& base,
    const PrescribedInterval& interval,double coefficient_thickness,ForceTrial& output) noexcept {
  if(r.input.placement!=ShellReferencePlacement::Centered) return Status::kInvalidInput;
  if(!detail::SaneReference(r)||!base.matches_reference(r)) return Status::kInvalidReference;
  const auto& stamp=base.stamp();
  if(!detail::ValidHistoryValues(base.data())||!tl::math::Finite(stamp.time)||stamp.time<0||
     interval.base_time!=stamp.time||stamp.sample_index==UINT64_MAX||
     interval.sample_index!=stamp.sample_index+1) return Status::kInvalidInput;
  detail::GeometryWork geometry;
  const auto status=detail::PrepareGeometry(r,interval,geometry);
  if(status!=Status::kSuccess) return status;
  ForceTrial candidate;
  candidate.kinematics=geometry.values; // Before native CNDT3 length mutation.
  detail::MaterialWork material;
  auto coefficient_input=r.input;
  coefficient_input.thickness=coefficient_thickness;
  if(!detail::PrepareMaterial(coefficient_input,geometry.values.area,interval.dt,material))
    return Status::kNonfiniteResult;
  auto proposed=base.data();
  if(!detail::UpdateLaw1(geometry,material,proposed)) return Status::kNonfiniteResult;
  detail::StiffnessDiagnostics(geometry,material,candidate.diagnostics);
  detail::LocalForceWork local;
  detail::ElasticForces(geometry,material,proposed,local);
  detail::StabilizationWork stabilization;
  detail::UpdateStabilization(geometry,material,proposed,stabilization);
  detail::StabilizationForces(geometry,material,proposed,stabilization,local);
  const auto projection_status=detail::ProjectForcesInWorkingLength(geometry,local,
      r.input.projection_working_length_m,candidate.internal_force,candidate.internal_couple);
  if(projection_status!=Status::kSuccess)return projection_status;
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
  output=candidate;
  return Status::kSuccess;
}
} // namespace detail

// Legacy fixed-coefficient-thickness API and ordinary shell history are unchanged.
TL_QEPH_HD inline Status EvaluateForce(const ReferenceData& r,const History& base,
    const PrescribedInterval& interval,ForceTrial& output) noexcept {
  return detail::EvaluateForceWithThickness(r,base,interval,r.input.thickness,output);
}

// Explicit native NPT0 analytic LAW1. No Gauss-point history is created.
// The physical owner must retain the resolved profile immutably across calls.
// CNCOEF3B applies its native EM20 length floor, converted explicitly to SI.
TL_QEPH_HD inline Status EvaluateGlobalLaw1Force(const ShellGlobalLaw1Profile& profile,
    const ReferenceData& r,const History& base,const PrescribedInterval& interval,
    ForceTrial& output) noexcept {
  if(!shell_global_law1::Valid(profile)) return Status::kInvalidInput;
  const double thickness=shell_global_law1::QephCoefficientThickness(profile,r.input.thickness,base.data().thickness);
  return detail::EvaluateForceWithThickness(r,base,interval,thickness,output);
}
} // namespace tl::fea::qeph
