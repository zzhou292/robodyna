// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "UnitPrepare.h"
#include "Force.h"
namespace tl::fea::solid_common::distortion {
TL_BRICK_HD inline Status ClassifyDamping(const PreparedForceValues& values,DampingActivity& output) noexcept {
  units_detail::Factors f;if(!units_detail::Make(values.units,f))return Status::UnsupportedProfile;
  return native::ClassifyDamping(values.parameters,values.input,output);
}
TL_BRICK_HD inline Status EvaluateForce(const PreparedForceValues& values,
    bool native_batch_damping_enabled,ForceResult& output) noexcept {
  units_detail::Factors f;if(!units_detail::Make(values.units,f))return Status::UnsupportedProfile;
  native::ForceResult raw;
  const auto status=native::EvaluateForce(values.parameters,values.input,native_batch_damping_enabled,raw);
  if(status!=Status::Success)return status;
  ForceResult next;
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)
    SetComponent(next.force_n[n],k,Component(raw.force[n],k)*f.base.force);
  next.raw_stiffness_n_m=raw.raw_stiffness*f.base.stiffness;
  next.distortion_energy_j=raw.distortion_energy*f.base.energy;
  next.distortion_work_increment_j=raw.distortion_work_increment*f.base.energy;
  next.damping_applied=raw.damping_applied;next.center_contacts=raw.center_contacts;next.corner_contacts=raw.corner_contacts;
  if(!tl::math::Finite(next.raw_stiffness_n_m)||!tl::math::Finite(next.distortion_energy_j)||
     !tl::math::Finite(next.distortion_work_increment_j))return Status::NonfiniteResult;
  for(const auto& force:next.force_n)if(!Finite(force))return Status::NonfiniteResult;
  output=next;return Status::Success;
}
} // namespace tl::fea::solid_common::distortion
