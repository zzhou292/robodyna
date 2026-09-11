// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected eight-point LAW90 S8EFORC3, OpenRadioss (C) 2026 Siemens.
#pragma once
#include "History.h"
#include "TotalKinematics.h"
#include "ForceWork.h"
#include "lib_src/elements/solid18/Solid18ForceValues.h"
#include "lib_src/materials/law90/Caller.h"
namespace tl::fea::solid18::total_strain {
namespace force_detail {
TL_SOLID18_HD inline Status Calculate(const Reference& reference,const Material& material,
    const History* accepted,const PrescribedInterval& interval,ForceScratch& scratch) noexcept {
  KinematicsInput input;
  input.dt_s=interval.dt_s;
  for(unsigned n=0;n<8;++n) {
    input.position_m[n]=interval.position_endpoint_m[n];
    input.velocity_m_s[n]=interval.velocity_midpoint_m_s[n];
  }
  auto status=EvaluateKinematics90Scratch(reference,input,scratch.kinematics);
  if(status!=Status::Success)return status;
  const auto& kinematics=scratch.kinematics.staged;
  auto& trial=scratch.staged;
  auto& next=scratch.next;
  next.global={}; // Exact S8ZZERO3 global reset; no selected previous-global use.
  trial.diagnostics={};
  trial.diagnostics.minimum_unscaled_dt_s=1e30;
  double length=1e30;
  for(auto& force:scratch.local_force_n)force={};
  for(unsigned r=0;r<2;++r)for(unsigned s=0;s<2;++s)for(unsigned t=0;t<2;++t) {
    const unsigned ip=r+2*s+4*t;
    const auto& geometry=kinematics.geometry.point[ip];
    // S8EDERI_2: running length is reduced before this point's MQVISCB.
    length=::fmin(length,128*geometry.current_volume_m3*
        kinematics.geometry.inverse_center_face_scale_per_m2/1.0);
    auto& observation=trial.point[ip];
    observation.current_volume_m3=geometry.current_volume_m3;
    observation.storage_volume_m3=reference.geometry().point[ip].initial_volume_m3;
    observation.characteristic_length_m=length;
    tl::material::law90::CallerInput caller;
    for(unsigned k=0;k<6;++k) {
      caller.selected_b_minus_identity[k]=kinematics.point[ip].selected_left_cauchy_green_minus_identity[k];
      caller.engineering_rate_per_s[k]=kinematics.point[ip].engineering_rate_per_s[k];
    }
    caller.endpoint_time_s=interval.base_time_s+interval.dt_s;
    caller.dt_s=interval.dt_s;
    caller.current_volume_m3=observation.current_volume_m3;
    caller.storage_volume_m3=observation.storage_volume_m3;
    caller.characteristic_length_m=length;
    const auto material_status=accepted ? tl::material::law90::UpdateCallerSI(material,
        accepted->data().point[ip],caller,observation.material) :
        tl::material::law90::InitializeCallerSI(material,caller,observation.material);
    if(material_status!=tl::material::law90::PointStatus::Ok)return Status::InvalidInput;
    next.point[ip]=observation.material.history;
    double stress[6];
    solid18::detail::PointStressVolume(next.point[ip].stress_pa,next.point[ip].bulk_pressure_pa,
        geometry.current_volume_m3,stress);
    solid18::detail::AccumulateSelectedShearForce(geometry,stress,scratch.local_force_n);
    AccumulateGlobal(reference,kinematics.geometry,ip,observation,next.global,trial.diagnostics);
  }
  for(unsigned n=0;n<8;++n) {
    const auto force=solid18::detail::WorldForce(kinematics.geometry.frame,scratch.local_force_n[n]);
    if(!solid18::detail::Finite(force))return Status::NonfiniteResult;
    trial.rhs_force_n[reference.source_slot(n)]=force;
  }
  if(!solid18::detail::Positive(trial.diagnostics.minimum_unscaled_dt_s)||
     !solid18::detail::Positive(trial.diagnostics.raw_stiffness_n_m)||
     !tl::math::Finite(trial.diagnostics.internal_work_increment_j))return Status::NonfiniteResult;
  return HistoryWriter::Prepare(reference,material,next,
      {interval.base_time_s+interval.dt_s,interval.sample_index},trial.proposed_history);
}
} // namespace force_detail
// Scratch is caller-owned and disjoint from all borrowed inputs and accepted
// destinations. Only success permits publication of scratch.staged.
TL_SOLID18_HD inline Status EvaluateForce90Scratch(const Reference& reference,
    const History& accepted,const PrescribedInterval& interval,const Material& material,
    ForceScratch& scratch) noexcept {
  if(!force_detail::ValidMaterial(reference,material)||!force_detail::ValidInterval(accepted,interval)||
     !force_detail::SameReference(reference,accepted.reference())||
     !tl::material::law90::SamePreparedMaterial(material,accepted.material())||
     !force_detail::ValidValues(material,accepted.data()))return Status::InvalidInput;
  return force_detail::Calculate(reference,material,&accepted,interval,scratch);
}
TL_SOLID18_HD inline Status InitializeForce90Scratch(const Reference& reference,
    const Material& material,Vec3 uniform_velocity_m_s,ForceScratch& scratch) noexcept {
  if(!force_detail::ValidMaterial(reference,material)||
     !solid18::detail::Finite(uniform_velocity_m_s))return Status::InvalidInput;
  PrescribedInterval initial;
  for(unsigned n=0;n<8;++n) {
    initial.position_endpoint_m[n]=reference.input().position_m[n];
    initial.velocity_midpoint_m_s[n]=uniform_velocity_m_s;
  }
  return force_detail::Calculate(reference,material,nullptr,initial,scratch);
}
inline Status InitializeForce90(const Reference& reference,const Material& material,
    Vec3 velocity,ForceTrial& output) noexcept {
  ForceScratch scratch;
  const auto status=InitializeForce90Scratch(reference,material,velocity,scratch);
  if(status==Status::Success)output=scratch.staged;
  return status;
}
inline Status EvaluateForce90(const Reference& reference,const History& accepted,
    const PrescribedInterval& interval,const Material& material,ForceTrial& output) noexcept {
  ForceScratch scratch;
  const auto status=EvaluateForce90Scratch(reference,accepted,interval,material,scratch);
  if(status==Status::Success)output=scratch.staged;
  return status;
}
} // namespace tl::fea::solid18::total_strain
