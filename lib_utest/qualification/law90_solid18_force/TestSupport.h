// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/solid18/total_strain/Force.h"
#include "lib_utest/qualification/law90_point/TestSupport.h"
#include "lib_utest/qualification/law90_solid18_reference/TestSupport.h"
namespace law90_force_test {
namespace s=tl::fea::solid18;
namespace f=s::total_strain;
namespace law=tl::material::law90;
using law90_test::Bytes;
inline law::PreparationInput ElementToyInput() {
  auto input=law90_point_test::ToyInput();input.reference_density_kg_m3=input.density_kg_m3;return input;
}
inline s::ReferenceInput Cube() {
  auto input=law90_reference_test::Cube();
  input.density_kg_m3=law90_test::OriginalInput().density_kg_m3;
  return input;
}
inline s::ReferenceInput Distorted() {
  auto input=law90_reference_test::Distorted();
  input.density_kg_m3=law90_test::OriginalInput().density_kg_m3;
  return input;
}
inline law::PreparedMaterial Material() {
  law::PreparedMaterial result;
  const auto status=law::PrepareSI(law90_test::OriginalInput(),law90_test::OriginalCurve(),result);
  EXPECT_EQ(status,law::Status::Ok);
  return result;
}
inline s::PrescribedInterval Path(const s::ReferenceInput& input,unsigned step,
    double dt=1e-5,double amplitude=.35,bool rotate=true) {
  s::PrescribedInterval result;
  result.base_time_s=(step-1)*dt;result.dt_s=dt;result.sample_index=step;
  const double omega=2*std::acos(-1.)/(80*dt),time=step*dt;
  const double wave=.5*(1-std::cos(omega*time)),wd=.5*omega*std::sin(omega*time);
  const double angle=rotate ? .005*step : 0,ad=rotate ? .005/dt : 0;
  const double c=std::cos(angle),sn=std::sin(angle);
  const auto origin=input.position_m[0];
  for(unsigned n=0;n<8;++n) {
    const auto p=input.position_m[n];
    const double x=p.x-origin.x,y=p.y-origin.y,z=p.z-origin.z;
    const double a=(1-amplitude*wave)*x+.03*wave*y;
    const double b=(1-.5*amplitude*wave)*y+.02*wave*z;
    const double d=(1-.2*amplitude*wave)*z+.01*wave*x;
    const double da=-amplitude*wd*x+.03*wd*y;
    const double db=-.5*amplitude*wd*y+.02*wd*z;
    const double dd=-.2*amplitude*wd*z+.01*wd*x;
    result.position_endpoint_m[n]={origin.x+c*a-sn*b+3*time,origin.y+sn*a+c*b-time,origin.z+d+2*time};
    result.velocity_midpoint_m_s[n]={c*da-sn*db-ad*(sn*a+c*b)+3,
                                   sn*da+c*db+ad*(c*a-sn*b)-1,dd+2};
  }
  return result;
}
inline void MatchBase(const f::History& h,s::PrescribedInterval& interval) {
  interval.base_time_s=h.stamp().time_s;interval.sample_index=h.stamp().sample_index+1;
}
inline void PackHistory(const law::CallerHistory& h,double* values) {
  law90_point_test::PackHistory(h.point,values);
  for(unsigned k=0;k<6;++k)values[10+k]=h.stress_pa[k];
  values[16]=h.density_kg_m3;values[17]=h.internal_energy_density_j_m3;
  values[18]=h.bulk_pressure_pa;values[19]=h.scalar_rate_per_s;
}
inline std::array<double,37> CallerValues(const law::CallerResult& r) {
  std::array<double,37> values{};PackHistory(r.history,values.data());
  for(unsigned k=0;k<6;++k)values[20+k]=r.point.cauchy_stress_pa[k];
  values[26]=r.point.sound_speed_m_s;values[27]=r.point.scalar_rate_s_inverse;
  values[28]=r.point.tangent_factor;values[29]=r.point.maximum_viscosity_pa_s;values[30]=r.point.active;
  values[31]=r.volume_increment_m3;values[32]=r.average_volume_m3;values[33]=r.density_compression;
  values[34]=r.internal_work_j;values[35]=r.unscaled_element_dt_s;values[36]=r.raw_stiffness_n_m;
  return values;
}
inline law::CallerInput CallerPath(unsigned step) {
  law::CallerInput input;
  const auto point=law90_point_test::Path(step,true);
  for(unsigned k=0;k<6;++k) {
    input.selected_b_minus_identity[k]=point.total_b_minus_i_engineering[k]/(k<3 ? 1.0 : 2.0);
    input.engineering_rate_per_s[k]=point.engineering_rate_s_inverse[k];
  }
  input.endpoint_time_s=step*1e-5;input.dt_s=step ? 1e-5 : 0;
  input.storage_volume_m3=.001;input.current_volume_m3=.001*(1-.3*std::sin(.017*step));
  input.characteristic_length_m=.04;
  return input;
}
}
