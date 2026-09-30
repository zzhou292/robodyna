// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "../Force.h"
#include "lib_src/elements/solid_common/distortion/NativeParameters.h"
#include "lib_src/elements/solid_common/distortion/Damping.h"
namespace tl::fea::solid18::total_strain::controlled_distortion {
namespace detail {
TL_SOLID18_HD inline bool SameUnits(distortion::UnitScale a,distortion::UnitScale b) noexcept {
  return a.length_m==b.length_m&&a.mass_kg==b.mass_kg&&a.time_s==b.time_s;
}
TL_SOLID18_HD inline distortion::Status Prepare(const Reference& reference,const History* accepted,
    const PrescribedInterval& interval,Scratch& scratch) noexcept {
  scratch.valid=false;
  const auto& material=reference.material();distortion::units_detail::Factors f;
  if(!reference.source().prepared()||!material.prepared()||
     !distortion::units_detail::Make(material.units(),f))return distortion::Status::InvalidInput;
  if(accepted&&(!accepted->prepared()||!SameUnits(accepted->units(),material.units())))
    return distortion::Status::InvalidInput;
  PrescribedInterval numeric=interval;
  numeric.base_time_s/=f.base.time;numeric.dt_s/=f.base.time;
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k) {
    solid_common::SetComponent(numeric.position_endpoint_m[n],k,
        solid_common::Component(interval.position_endpoint_m[n],k)/f.base.length);
    solid_common::SetComponent(numeric.velocity_midpoint_m_s[n],k,
        solid_common::Component(interval.velocity_midpoint_m_s[n],k)/f.base.velocity);
  }
  const auto status=accepted?EvaluateForce90Scratch(reference.native_reference(),accepted->native_history(),
      numeric,material.native_material(),scratch.force):
      InitializeForce90Scratch(reference.native_reference(),material.native_material(),
          numeric.velocity_midpoint_m_s[0],scratch.force);
  if(status!=Status::Success)return distortion::Status::InvalidInput;
  auto& next=scratch.distortion;next.units=material.units();
  const auto& before=scratch.force.staged;
  const auto& global=before.proposed_history.data().global;const auto& last=before.point[7];
  const auto& p=material.slots();
  const auto parameters=distortion::native::PrepareParameters(p.pm21_poisson_ratio,
      p.pm22_shear_pa,p.pm32_bulk_pa,p.pm100_reader_contact_bulk_pa,p.pm107_control_pa,
      global.stress_pa,global.density_kg_m3,last.material.point.sound_speed_m_s,
      last.current_volume_m3,next.parameters);
  if(parameters!=distortion::Status::Success)return parameters;
  for(unsigned n=0;n<8;++n) {
    const auto source=reference.native_reference().source_slot(n);
    next.input.position[n]=numeric.position_endpoint_m[source];
    next.input.velocity[n]=numeric.velocity_midpoint_m_s[source];
    next.input.incoming_force[n]=before.rhs_force_n[source];
  }
  next.input.dt=numeric.dt_s;
  next.input.raw_stiffness=last.material.raw_stiffness_n_m;
  next.input.distortion_energy=accepted?accepted->native_distortion_energy():0;
  const auto classify=distortion::native::ClassifyDamping(next.parameters,next.input,scratch.activity);
  if(classify!=distortion::Status::Success)return classify;
  scratch.valid=true;return distortion::Status::Success;
}
} // namespace detail
TL_SOLID18_HD inline distortion::Status PrepareInitial(const Reference& reference,Vec3 velocity,
                                                       Scratch& scratch) noexcept {
  PrescribedInterval interval;
  for(unsigned n=0;n<8;++n) {
    interval.position_endpoint_m[n]=reference.source().input().position_m[n];
    interval.velocity_midpoint_m_s[n]=velocity;
  }
  return detail::Prepare(reference,nullptr,interval,scratch);
}
TL_SOLID18_HD inline distortion::Status PrepareCandidate(const Reference& reference,
    const History& accepted,const PrescribedInterval& interval,Scratch& scratch) noexcept {
  return detail::Prepare(reference,&accepted,interval,scratch);
}
} // namespace tl::fea::solid18::total_strain::controlled_distortion
