// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_src/elements/solid_common/distortion/NativeParameters.h"
#include "lib_src/elements/solid_common/distortion/Damping.h"
namespace tl::fea::solid24::controlled_distortion {
namespace detail {
TL_BRICK_HD inline ForceStatus Map(distortion::Status s)noexcept {
  if(s==distortion::Status::Success)return ForceStatus::Success;
  if(s==distortion::Status::UnsupportedProfile)return ForceStatus::UnsupportedProfile;
  if(s==distortion::Status::NonfiniteResult)return ForceStatus::NonfiniteResult;
  return ForceStatus::InvalidInput;
}
TL_BRICK_HD inline bool SameUnits(distortion::UnitScale a,distortion::UnitScale b)noexcept {
  return a.length_m==b.length_m&&a.mass_kg==b.mass_kg&&a.time_s==b.time_s;
}
TL_BRICK_HD inline ForceStatus Prepare(const Reference& reference,const History* accepted,
    const PrescribedInterval& interval,Scratch& scratch)noexcept {
  scratch.valid=false;hour::units_detail::Factors f;
  if(!reference.prepared()||!hour::units_detail::Make(reference.units(),f))return ForceStatus::InvalidInput;
  if(!tl::math::Finite(interval.base_time_s)||!tl::math::Finite(interval.dt_s))return ForceStatus::InvalidInput;
  if(accepted){
    if(!accepted->prepared()||!SameUnits(accepted->units(),reference.units())||
       !force_detail::SameReference(accepted->reference(),reference.reference())||
       !force_detail::SameMaterial(accepted->material(),reference.material()))return ForceStatus::ReferenceMismatch;
    if(interval.base_time_s!=accepted->stamp().time_s||interval.dt_s<=0||
       !tl::math::Finite(interval.base_time_s+interval.dt_s)||interval.base_time_s+interval.dt_s<=interval.base_time_s||
       accepted->stamp().sample_index==UINT64_MAX||interval.sample_index!=accepted->stamp().sample_index+1)
      return ForceStatus::InvalidInput;
  }else if(interval.base_time_s!=0||interval.dt_s!=0||interval.sample_index!=0)return ForceStatus::InvalidInput;
  auto numeric=interval;numeric.base_time_s/=f.base.time;numeric.dt_s/=f.base.time;
  for(unsigned n=0;n<8;++n){
    if(!solid_common::Finite(interval.position_m[n])||!solid_common::Finite(interval.velocity_m_s[n]))return ForceStatus::InvalidInput;
    numeric.position_m[n]=hour::units_detail::Divide(interval.position_m[n],f.base.length);
    numeric.velocity_m_s[n]=hour::units_detail::Divide(interval.velocity_m_s[n],f.base.velocity);
  }
  hour::HistoryValues virgin;virgin.material.density_kg_m3=reference.native_material().density_kg_m3;
  const auto& old=accepted?accepted->native_values():virgin;
  // Native material/FHOUR history is never converted through SI between steps.
  auto status=hour::working_detail::EvaluateNumeric(reference,old.material,old.controlled_hourglass,numeric,
      accepted==nullptr,scratch.prefix);
  if(status!=ForceStatus::Success)return status;
  scratch.reference=reference;scratch.proposed_stamp={interval.base_time_s+interval.dt_s,interval.sample_index};
  tl::material::law42::MechanicalSlots slots;
  if(tl::material::law42::PrepareMechanicalSlots(reference.native_material(),slots)!=tl::material::law42::Status::Ok)
    return ForceStatus::MaterialFailure;
  auto& next=scratch.distortion;next.units=reference.units();const auto& m=scratch.prefix.material;
  status=Map(distortion::native::PrepareParameters(slots.pm21_poisson_ratio,slots.pm22_gs_pa,
      slots.pm32_pa,slots.pm100_reader_bulk_pa,slots.pm107_control_pa,m.history.stress_pa,
      m.history.density_kg_m3,m.point.sound_speed_m_s,scratch.prefix.geometry.current.volume_m3,next.parameters));
  if(status!=ForceStatus::Success)return status;
  for(unsigned n=0;n<8;++n){const auto source=reference.native_reference().source_slot(n);
    next.input.position[n]=numeric.position_m[source];next.input.velocity[n]=numeric.velocity_m_s[source];
    next.input.incoming_force[n]=scratch.prefix.stage.world_native_force_before_distortion_n[n];}
  next.input.dt=numeric.dt_s;next.input.raw_stiffness=scratch.prefix.stage.raw_stiffness_before_distortion_n_m;
  next.input.distortion_energy=accepted?accepted->native_distortion_energy():0;
  status=Map(distortion::native::ClassifyDamping(next.parameters,next.input,scratch.activity));
  if(status!=ForceStatus::Success)return status;scratch.valid=true;return ForceStatus::Success;
}
} // namespace detail
TL_BRICK_HD inline ForceStatus PrepareInitial(const Reference& reference,Vec3 velocity,Scratch& scratch)noexcept {
  PrescribedInterval i;for(unsigned n=0;n<8;++n){i.position_m[n]=reference.reference().input().position_m[n];i.velocity_m_s[n]=velocity;}
  return detail::Prepare(reference,nullptr,i,scratch);
}
TL_BRICK_HD inline ForceStatus PrepareCandidate(const Reference& reference,const History& accepted,
    const PrescribedInterval& interval,Scratch& scratch)noexcept{return detail::Prepare(reference,&accepted,interval,scratch);}
} // namespace tl::fea::solid24::controlled_distortion
