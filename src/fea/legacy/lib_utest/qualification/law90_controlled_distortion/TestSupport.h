// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid18/total_strain/controlled_distortion/Stage.h"
#include "lib_utest/qualification/law90_solid18_force/NativeSupport.h"
namespace law90_control_test {
namespace c=tl::fea::solid18::total_strain::controlled_distortion;
namespace d=c::distortion;
namespace f=tl::fea::solid18::total_strain;
namespace s=tl::fea::solid18;
namespace b=tl::fea::solid_common;
namespace law=tl::material::law90;
using law90_force_test::MatchBase;
inline d::units_detail::Factors Factors(d::UnitScale units) {
  d::units_detail::Factors out;EXPECT_TRUE(d::units_detail::Make(units,out));return out;
}
inline law::PreparationInput WorkingInput(law::PreparationInput input,d::UnitScale units) {
  const auto f=Factors(units);const double density=f.base.mass/f.volume;
  input.density_kg_m3/=density;input.reference_density_kg_m3/=density;
  input.card_young_pa/=f.pressure;input.contact_modulus_pa/=f.pressure;
  input.tension_cutoff_pa/=f.pressure;input.curve_scale_dimension/=f.pressure;
  if(input.curve_scale!=0)input.curve_scale/=f.pressure;
  return input;
}
inline s::ReferenceInput WorkingReference(s::ReferenceInput input,d::UnitScale units) {
  const auto f=Factors(units);input.density_kg_m3/=f.base.mass/f.volume;
  for(auto& x:input.position_m)for(unsigned k=0;k<3;++k)b::SetComponent(x,k,b::Component(x,k)/f.base.length);
  return input;
}
inline s::PrescribedInterval WorkingInterval(s::PrescribedInterval input,d::UnitScale units) {
  const auto f=Factors(units);
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k) {
    b::SetComponent(input.position_endpoint_m[n],k,b::Component(input.position_endpoint_m[n],k)/f.base.length);
    b::SetComponent(input.velocity_midpoint_m_s[n],k,b::Component(input.velocity_midpoint_m_s[n],k)/f.base.velocity);
  }
  input.dt_s/=f.base.time;input.base_time_s/=f.base.time;return input;
}
inline s::PrescribedInterval Path(const s::ReferenceInput& reference,unsigned step) {
  auto interval=law90_force_test::Path(reference,step,2e-7,.08,true);
  // Nonuniform represented velocities exercise SFOR_VISN8 without editing stress.
  for(unsigned n=0;n<8;++n) {
    const double sign=n%2?1:-1;
    interval.velocity_midpoint_m_s[n]={sign+.01,2*sign+.02,3*sign+.01};
  }
  return interval;
}
inline c::Reference Reference(const s::ReferenceInput& input,const law::PreparedMaterial& material,d::UnitScale units) {
  f::Reference original;EXPECT_EQ(f::InitializeReference90(input,original),s::Status::Success);
  c::Material control;EXPECT_EQ(c::PrepareMaterial(material,units,control),d::Status::Success);
  c::Reference result;EXPECT_EQ(c::PrepareReference(original,control,result),d::Status::Success);return result;
}
inline void Compare(const c::Result& actual,const c::Result& expected) {
  double scale=0;for(const auto& x:expected.rhs_force_n)for(unsigned k=0;k<3;++k)
    scale=::fmax(scale,::fabs(b::Component(x,k)));
  for(unsigned n=0;n<8;++n)for(unsigned k=0;k<3;++k)
    EXPECT_NEAR(b::Component(actual.rhs_force_n[n],k),b::Component(expected.rhs_force_n[n],k),2e-10*scale)<<n<<':'<<k;
  const double a[]{actual.nodal_raw_stiffness_n_m,actual.last_point_raw_stiffness_after_distortion_n_m,
    actual.distortion_energy_j,actual.distortion_work_increment_j,actual.minimum_unscaled_dt_s,actual.material_work_increment_j};
  const double e[]{expected.nodal_raw_stiffness_n_m,expected.last_point_raw_stiffness_after_distortion_n_m,
    expected.distortion_energy_j,expected.distortion_work_increment_j,expected.minimum_unscaled_dt_s,expected.material_work_increment_j};
  for(unsigned k=0;k<6;++k)EXPECT_NEAR(a[k],e[k],2e-10*::fabs(e[k]))<<k;
}
} // namespace law90_control_test
