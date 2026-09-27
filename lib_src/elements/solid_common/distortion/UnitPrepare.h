// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "UnitTypes.h"
#include "NativeParameters.h"
namespace tl::fea::solid_common::distortion {
TL_BRICK_HD inline Status PrepareForceValues(const tl::material::law42::Parameters& material,
    const Input& parameter_input,const ForceInput& input,UnitScale units,
    PreparedForceValues& output) noexcept {
  units_detail::Factors f;
  if(!units_detail::Make(units,f))return Status::UnsupportedProfile;
  Parameters admitted;
  const Status status=PrepareParameters(material,parameter_input,admitted);
  if(status!=Status::Success)return status;
  PreparedForceValues next;next.units=units;
  if(units.length_m==1) {
    next.parameters={admitted.length_m,admitted.damping_n_s_m2,admitted.control_stiffness_n_m,
      admitted.damping_coefficient,admitted.quadratic_limit,admitted.buckling_flag};
  } else {
    // Re-evaluate authenticated reader slot expressions in native numeric units.
    // These temporaries never escape the unit boundary as SI material objects.
    tl::material::law42::Parameters numeric_material;
    if(tl::material::law42::Prepare(material.mu_pa/f.pressure,material.poisson_ratio,
        material.density_kg_m3/(f.base.mass/f.volume),material.tension_cutoff_pa/f.pressure,
        numeric_material)!=tl::material::law42::Status::Ok)return Status::NonfiniteResult;
    tl::material::law42::MechanicalSlots slots;
    if(tl::material::law42::PrepareMechanicalSlots(numeric_material,slots)!=tl::material::law42::Status::Ok)return Status::NonfiniteResult;
    double sig[6];for(unsigned k=0;k<6;++k)sig[k]=parameter_input.cauchy_stress_pa[k]/f.pressure;
    const auto native_status=native::PrepareParameters(slots.pm21_poisson_ratio,slots.pm22_gs_pa,
      slots.pm32_pa,slots.pm100_reader_bulk_pa,slots.pm107_control_pa,sig,
      parameter_input.density_kg_m3/(f.base.mass/f.volume),parameter_input.material_sound_speed_m_s/f.base.velocity,
      parameter_input.current_volume_m3/f.volume,next.parameters);
    if(native_status!=Status::Success)return native_status;
  }
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k) {
    SetComponent(next.input.position[n],k,Component(input.position_m[n],k)/f.base.length);
    SetComponent(next.input.velocity[n],k,Component(input.velocity_m_s[n],k)/f.base.velocity);
    SetComponent(next.input.incoming_force[n],k,Component(input.incoming_force_n[n],k)/f.base.force);
  }
  next.input.dt=input.dt_s/f.base.time;
  next.input.raw_stiffness=input.raw_stiffness_n_m/f.base.stiffness;
  next.input.distortion_energy=input.distortion_energy_j/f.base.energy;
  if(!native::detail::Valid(next.parameters,next.input))return Status::InvalidInput;
  output=next;return Status::Success;
}
} // namespace tl::fea::solid_common::distortion
