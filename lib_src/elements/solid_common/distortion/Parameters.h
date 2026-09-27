// SPDX-License-Identifier: AGPL-3.0-or-later
// SDISTOR_INI / SCRE_SIG3, OpenRadioss a62b27e6 (C) 2026 Siemens.
#pragma once
#include "Types.h"
namespace tl::fea::solid_common::distortion {
// Source constants use WP expressions, while bare .4/.48999 are default REAL
// widened to WP. Preserve the source arithmetic instead of substituting 4/3.
inline constexpr double NativeOneP333=((1.0+3.0/10.0)+3.0/100.0)+3.0/1000.0;
TL_LAW42_HD inline Status PrepareParameters(const tl::material::law42::Parameters& material,
                                           const Input& input,Parameters& output) noexcept {
  tl::material::law42::MechanicalSlots slots;
  const auto slot_status=tl::material::law42::PrepareMechanicalSlots(material,slots);
  if(slot_status==tl::material::law42::Status::NonfiniteResult)return Status::NonfiniteResult;
  if(slot_status!=tl::material::law42::Status::Ok)return Status::InvalidMaterial;
  if(!tl::math::Finite(input.density_kg_m3)||input.density_kg_m3<=0 ||
     !tl::math::Finite(input.material_sound_speed_m_s)||input.material_sound_speed_m_s<=0 ||
     !tl::math::Finite(input.current_volume_m3)||input.current_volume_m3<=0 ||
     !tl::math::Finite(input.off)||!tl::math::Finite(input.offg))return Status::InvalidInput;
  for(double value:input.cauchy_stress_pa)if(!tl::math::Finite(value))return Status::InvalidInput;
  if(input.units!=WorkingUnits::SI || input.off!=1 || input.offg!=1 || input.ismstr!=10 ||
     slots.pm21_poisson_ratio<0 || slots.pm21_poisson_ratio>MaximumPoissonRatio)
    return Status::UnsupportedProfile;
  Parameters next;
  next.damping_coefficient=5.0/100.0;
  next.quadratic_limit=100.0;
  double f_nu=1.0;
  if(slots.pm21_poisson_ratio>static_cast<double>(0.4f)) {
    f_nu=1.0-2.0*slots.pm21_poisson_ratio;
    next.damping_coefficient=f_nu*next.damping_coefficient;
  } else if(slots.pm107_control_pa>=180.0*slots.pm32_pa) {
    f_nu=10.0;
  }
  const double c1=::fmax(slots.pm32_pa,slots.pm100_reader_bulk_pa)+NativeOneP333*slots.pm22_gs_pa;
  const double c2=f_nu*c1;
  const auto& sig=input.cauchy_stress_pa;
  const double aj2=.5*(sig[0]*sig[0]+sig[1]*sig[1]+sig[2]*sig[2])+
                    sig[3]*sig[3]+sig[4]*sig[4]+sig[5]*sig[5];
  if(!tl::math::Finite(c1)||!tl::math::Finite(c2)||!tl::math::Finite(aj2))return Status::NonfiniteResult;
  const double es=::sqrt(3.0*aj2)/c1;
  if(!tl::math::Finite(es))return Status::NonfiniteResult;
  double f_es=::fmax(1.0/1000.0,100.0*es);
  f_es=::fmin(1.0,f_es);
  next.length_m=::pow(input.current_volume_m3,1.0/3.0);
  const double caq=f_es*next.damping_coefficient*input.density_kg_m3*next.length_m;
  next.damping_n_s_m2=.25*caq*input.material_sound_speed_m_s*input.off;
  next.control_stiffness_n_m=c2*next.length_m*input.off;
  // SCRE_SIG3 does not diagonalize SIG. Only its active ISMSTR10 branch is
  // admitted here; skip/ISMSTR12 behavior remains unsupported.
  double minimum=sig[0];
  for(unsigned i=1;i<6;++i)minimum=::fmin(minimum,sig[i]);
  next.buckling_flag=c1<-minimum?1:0;
  if(!tl::math::Finite(next.length_m)||!tl::math::Finite(next.damping_n_s_m2)||
     !tl::math::Finite(next.control_stiffness_n_m))return Status::NonfiniteResult;
  output=next;
  return Status::Success;
}
} // namespace tl::fea::solid_common::distortion
