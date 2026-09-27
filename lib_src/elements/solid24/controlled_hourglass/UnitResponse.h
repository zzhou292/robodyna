// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "UnitConversions.h"
#include "lib_src/elements/solid24/Solid24ForceKinematics.h"
#include "lib_src/elements/solid24/Solid24ForceMaterial.h"
namespace tl::fea::solid24::controlled_hourglass {
TL_BRICK_HD inline ForceStatus PrepareWorkingReference(const Reference& reference,const Material& material,
    UnitScale units,WorkingReference& output)noexcept {
  units_detail::Factors f;
  if(!units_detail::Make(units,f)||!reference.prepared()||!reference.reference_jacobian()||
     material.poisson_ratio<0||material.poisson_ratio>hg::MaximumPoissonRatio)
    return ForceStatus::UnsupportedProfile;
  const auto expected=units.length_m==1?WorkingLengthUnit::Metre:WorkingLengthUnit::Millimetre;
  if(reference.input().profile.working_length!=expected)return ForceStatus::UnsupportedProfile;
  if(!force_detail::ValidMaterial(reference,material))return ForceStatus::InvalidInput;
  WorkingReference next;next.physical_reference_=reference;next.physical_material_=material;next.units_=units;
  // Private numerical temporaries execute every source literal in native units.
  // Metre here selects unit-one literal floors on native coordinates; the
  // original physical reference and its actual unit stamp are retained above.
  auto input=reference.input();input.profile.working_length=WorkingLengthUnit::Metre;
  for(auto& x:input.position_m)x=units_detail::Divide(x,f.base.length);
  input.density_kg_m3/=f.base.mass/f.volume;
  const auto reference_status=InitializeReference(input,next.numerical_reference_);
  if(reference_status==Status::UnsupportedProfile)return ForceStatus::UnsupportedProfile;
  if(reference_status==Status::InvalidInput)return ForceStatus::InvalidInput;
  if(reference_status!=Status::Success)return ForceStatus::InvalidGeometry;
  if(tl::material::law42::Prepare(material.mu_pa/f.pressure,material.poisson_ratio,
      material.density_kg_m3/(f.base.mass/f.volume),material.tension_cutoff_pa/f.pressure,
      next.numerical_material_)!=tl::material::law42::Status::Ok)return ForceStatus::MaterialFailure;
  next.prepared_=true;output=next;return ForceStatus::Success;
}
TL_BRICK_HD inline ForceStatus EvaluateWorking(const WorkingReference& reference,const HistoryValues& accepted,
    const PrescribedInterval& interval,bool initialization,WorkingResult& output)noexcept {
  if(!reference.prepared_)return ForceStatus::InvalidInput;
  units_detail::Factors f;if(!units_detail::Make(reference.units_,f))return ForceStatus::UnsupportedProfile;
  if(!tl::math::Finite(interval.base_time_s)||!tl::math::Finite(interval.dt_s)||
     !(initialization?interval.dt_s==0:interval.dt_s>0))return ForceStatus::InvalidInput;
  auto native_interval=interval;
  for(unsigned n=0;n<8;++n){
    if(!solid_common::Finite(interval.position_m[n])||!solid_common::Finite(interval.velocity_m_s[n]))return ForceStatus::InvalidInput;
    native_interval.position_m[n]=units_detail::Divide(interval.position_m[n],f.base.length);
    native_interval.velocity_m_s[n]=units_detail::Divide(interval.velocity_m_s[n],f.base.velocity);
  }
  native_interval.dt_s/=f.base.time;native_interval.base_time_s/=f.base.time;
  const auto old_material=units_detail::ToNative(accepted.material,f);auto old_hourglass=accepted.controlled_hourglass;
  for(auto& row:old_hourglass.force_n)for(double& v:row)v/=f.base.force;
  WorkingResult next;
  auto status=force_detail::CurrentKinematics(reference.numerical_reference_,native_interval,next.geometry);
  if(status!=ForceStatus::Success)return status;
  status=force_detail::EvaluateMaterial(reference.numerical_reference_,reference.numerical_material_,
      next.geometry,native_interval.dt_s,old_material,initialization,next.material);
  if(status!=ForceStatus::Success)return status;
  status=EvaluateBeforeDistortion(reference.numerical_reference_,reference.numerical_material_,next.geometry,
      next.material,native_interval.dt_s,old_hourglass,next.stage);
  if(status!=ForceStatus::Success)return status;
  next.native_modal_work.units=reference.units_;next.native_modal_work.work=next.stage.hourglass.work_j;
  for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h){
    next.native_modal_work.rate[k][h]=next.stage.hourglass.modal_velocity_m_s[k][h];
    next.native_modal_work.force[k][h]=next.stage.hourglass.modal_force_n[k][h];
  }
  units_detail::GeometryToSi(next.geometry,f);units_detail::MaterialToSi(next.material,f);
  units_detail::StageToSi(next.stage,f);
  if(!units_detail::Finite(next))return ForceStatus::NonfiniteResult;
  output=next;return ForceStatus::Success;
}
} // namespace tl::fea::solid24::controlled_hourglass
