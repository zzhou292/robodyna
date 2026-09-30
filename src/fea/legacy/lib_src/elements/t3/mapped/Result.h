// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../T3History.h"
#include "../T3ForceData.h"

namespace tl::fea::t3::mapped {
TL_T3_HD inline bool Vector(Vec3 value,bool zero) noexcept {
  return zero ? value.x==0&&value.y==0&&value.z==0 :
      tl::math::Finite(value.x)&&tl::math::Finite(value.y)&&tl::math::Finite(value.z);
}
TL_T3_HD inline bool KinematicValues(const Kinematics& k,bool zero) noexcept {
  for (double value:k.frame.v) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (double value:k.derivative) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (double value:k.raw_rate) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (double value:k.normalized_rate) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (double value:k.corrected_velocity_difference) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (auto value:k.local_position) if (!Vector(value,zero)) return false;
  const double scalar[]{k.area,k.characteristic_length,k.area_scale,k.base_time,k.position_time,k.velocity_time,k.dt};
  for (double value:scalar) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  return !zero || !k.valid;
}
// Rigid skins carry only source/endpoint bookkeeping. Their reserved force,
// kinematic and section slots are unavailable mechanics, never a native zero
// deformation packet. Initial constitutive caches use the same zero storage.
TL_T3_HD inline bool ValidResult(const ReferenceData& reference,const ForceTrial& result,
    double time,std::uint64_t epoch,bool skin) noexcept {
  const auto& history=result.proposed_history;
  if (!history.matches_reference(reference) || history.stamp().time!=time ||
      history.stamp().sample_index!=epoch) return false;
  const bool zero=skin||epoch==0;
  const auto& h=history.data();
  if (!detail::ValidHistoryValues(h,true) || (zero &&
      (h.active!=1 || h.thickness!=reference.input.thickness))) return false;
  for (double value:h.stress) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (double value:h.material_stress) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (double value:h.bending_stress) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (double value:h.strain_curvature) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  for (double value:h.internal_work) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  if (zero ? h.equivalent_strain_rate!=0 : !tl::math::Finite(h.equivalent_strain_rate)) return false;
  for (auto value:result.internal_force) if (!Vector(value,zero)) return false;
  for (auto value:result.internal_couple) if (!Vector(value,zero)) return false;
  const auto& d=result.diagnostics;
  const double diagnostics[]{d.effective_thickness,d.native_sound_speed,d.membrane_viscosity,d.shear_factor,d.transverse_shear_modulus,d.translational_stiffness,d.rotational_stiffness,d.unscaled_element_dt,d.internal_work_increment[0],d.internal_work_increment[1]};
  for (double value:diagnostics) if (zero ? value!=0 : !tl::math::Finite(value)) return false;
  if (!KinematicValues(result.kinematics,zero)) return false;
  if (zero) return result.kinematics.sample_index==0;
  return result.kinematics.sample_index==epoch &&
      result.kinematics.base_time+result.kinematics.dt==time &&
      result.kinematics.area>0 && d.translational_stiffness>=0 && d.rotational_stiffness>=0;
}
TL_T3_HD inline Status AdvanceSkin(const ReferenceData& reference,const ForceTrial& base,
    const PrescribedInterval& interval,ForceTrial& output) noexcept {
  if (!interval.sample_index ||
      !ValidResult(reference,base,interval.base_time,interval.sample_index-1,true) || !tl::math::Finite(interval.dt) || interval.dt<=0 ||
      !tl::math::Finite(interval.base_time+interval.dt) || interval.base_time+interval.dt<=interval.base_time) {
    return Status::kInvalidInput;
  }
  ForceTrial next;
  const auto status=InitializeHistory(reference,{interval.base_time+interval.dt,interval.sample_index},
      next.proposed_history);
  if (status==Status::kSuccess) output=next;
  return status;
}
} // namespace tl::fea::t3::mapped
