// SPDX-License-Identifier: AGPL-3.0-or-later
// S8EFORC3: complete native-unit force path and separate STI/STIN publication.
#pragma once
#include "Prepare.h"
#include "lib_src/elements/solid_common/distortion/Force.h"
namespace tl::fea::solid18::total_strain::controlled_distortion {
TL_SOLID18_HD inline distortion::Status Complete(const Scratch& scratch,
    bool native_batch_damping_enabled,Result& output) noexcept {
  if(!scratch.valid)return distortion::Status::InvalidInput;
  distortion::units_detail::Factors f;
  if(!distortion::units_detail::Make(scratch.distortion.units,f))return distortion::Status::UnsupportedProfile;
  distortion::native::ForceResult force;
  const auto status=distortion::native::EvaluateForce(scratch.distortion.parameters,
      scratch.distortion.input,native_batch_damping_enabled,force);
  if(status!=distortion::Status::Success)return status;
  const auto& before=scratch.force.staged;Result next;
  const auto& reference=before.proposed_history.reference();
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)
    solid_common::SetComponent(next.rhs_force_n[reference.source_slot(n)],k,
        solid_common::Component(force.force[n],k)*f.base.force);
  next.nodal_raw_stiffness_n_m=before.diagnostics.raw_stiffness_n_m*f.base.stiffness;
  next.last_point_raw_stiffness_after_distortion_n_m=force.raw_stiffness*f.base.stiffness;
  next.minimum_unscaled_dt_s=before.diagnostics.minimum_unscaled_dt_s*f.base.time;
  next.material_work_increment_j=before.diagnostics.internal_work_increment_j*f.base.energy;
  next.distortion_energy_j=force.distortion_energy*f.base.energy;
  next.distortion_work_increment_j=force.distortion_work_increment*f.base.energy;
  const double values[]{next.nodal_raw_stiffness_n_m,next.last_point_raw_stiffness_after_distortion_n_m,
    next.minimum_unscaled_dt_s,next.material_work_increment_j,next.distortion_energy_j,next.distortion_work_increment_j};
  for(double value:values)if(!tl::math::Finite(value))return distortion::Status::NonfiniteResult;
  for(const auto& value:next.rhs_force_n)if(!solid_common::Finite(value))return distortion::Status::NonfiniteResult;
  HistoryWriter::Set(before.proposed_history,force.distortion_energy,scratch.distortion.units,next.proposed_history);
  output=next;return distortion::Status::Success;
}
} // namespace tl::fea::solid18::total_strain::controlled_distortion
