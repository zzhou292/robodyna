// SPDX-License-Identifier: AGPL-3.0-or-later
// Complete selected controlled HEPH numerical force before owner assembly.
#pragma once
#include "Prepare.h"
#include "lib_src/elements/solid_common/distortion/Force.h"
namespace tl::fea::solid24::controlled_distortion {
TL_BRICK_HD inline ForceStatus Complete(const Scratch& scratch,bool native_batch_damping_enabled,Result& output)noexcept {
  if(!scratch.valid)return ForceStatus::InvalidInput;
  hour::units_detail::Factors f;if(!hour::units_detail::Make(scratch.reference.units(),f))return ForceStatus::UnsupportedProfile;
  distortion::native::ForceResult force;
  const auto status=detail::Map(distortion::native::EvaluateForce(scratch.distortion.parameters,scratch.distortion.input,
      native_batch_damping_enabled,force));
  if(status!=ForceStatus::Success)return status;
  Result next;for(unsigned n=0;n<8;++n)
    next.rhs_force_n[scratch.reference.native_reference().source_slot(n)]=hour::units_detail::Multiply(force.force[n],f.base.force);
  const auto& p=scratch.prefix;
  next.material_raw_stiffness_n_m=p.material.raw_stiffness_n_m*f.base.stiffness;
  next.hourglass_raw_stiffness_n_m=p.stage.raw_stiffness_before_distortion_n_m*f.base.stiffness;
  next.nodal_raw_stiffness_n_m=force.raw_stiffness*f.base.stiffness;
  next.minimum_unscaled_dt_s=p.material.unscaled_element_dt_s*f.base.time;
  next.material_work_increment_j=p.material.internal_work_j*f.base.energy;
  next.hourglass_work_increment_j=p.stage.hourglass.work_j*f.base.energy;
  next.distortion_energy_j=force.distortion_energy*f.base.energy;
  next.distortion_work_increment_j=force.distortion_work_increment*f.base.energy;
  next.internal_energy_density_j_m3=p.stage.proposed_values.material.internal_energy_density_j_m3*f.pressure;
  next.damping_applied=force.damping_applied;next.center_contacts=force.center_contacts;next.corner_contacts=force.corner_contacts;
  const double values[]{next.material_raw_stiffness_n_m,next.hourglass_raw_stiffness_n_m,next.nodal_raw_stiffness_n_m,
    next.minimum_unscaled_dt_s,next.material_work_increment_j,next.hourglass_work_increment_j,
    next.distortion_energy_j,next.distortion_work_increment_j,next.internal_energy_density_j_m3};
  for(double value:values)if(!tl::math::Finite(value))return ForceStatus::NonfiniteResult;
  for(const auto& v:next.rhs_force_n)if(!solid_common::Finite(v))return ForceStatus::NonfiniteResult;
  HistoryWriter::Set(scratch,force.distortion_energy,next.proposed_history);output=next;return ForceStatus::Success;
}
} // namespace tl::fea::solid24::controlled_distortion
