// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solid24_force/NativeOracle.h"
#include "native/Packets.h"
#include <cmath>
#include <limits>
extern "C" void ic1_native_slots(const double*,double*);
extern "C" void ic1_native_geometry_counts(int*,int*);
extern "C" void ic1_native_distortion_observation(double*,double*,int*);
extern "C" void ic1_native_parameter_probe(const double*,const double*,const double*,double*,int*);
extern "C" void ic1_force_native(const double*,const double*,const double*,const double*,const double*,
  const double*,const double*,const double*,double*,std::int64_t*,int*);
namespace solid24_icontrol_test {
namespace s=tl::fea::solid24;
struct History {
  heph_test::NativeHistory reference;
  std::array<double,22> values{};
  explicit History(const s::ReferenceInput& source):reference(heph_test::InitializeNative(source)) {
    std::copy_n(reference.values.begin(),9,values.begin());
  }
};
struct Trial {
  std::array<double,93> values{};
  std::int64_t stages=0;
  int status=-1,center_contacts=0,corner_contacts=0,distortion_flag=0;
  std::array<double,6> distortion_sigma{};
  std::array<double,5> distortion_parameters{};
};
inline Trial Step(const History& history,const s::PrescribedInterval& interval,const s::Material& material) {
  std::array<double,24> x{},v{};
  for(unsigned n=0;n<8;++n) {
    const auto source=history.reference.reference.permutation[n];
    const auto& p=interval.position_m[source];const auto& u=interval.velocity_m_s[source];
    x[3*n]=p.x;x[3*n+1]=p.y;x[3*n+2]=p.z;v[3*n]=u.x;v[3*n+1]=u.y;v[3*n+2]=u.z;
  }
  const double p[]{material.mu_pa,material.poisson_ratio,material.density_kg_m3,material.tension_cutoff_pa};
  const double time[]{interval.base_time_s,interval.dt_s};auto jac=history.reference.reference.jacobian();
  const double volume=history.reference.reference.values[33];Trial result;result.values.fill(-9876.25);
  ic1_force_native(p,history.reference.initial.data(),x.data(),v.data(),jac.data(),&volume,
    history.values.data(),time,result.values.data(),&result.stages,&result.status);
  ic1_native_geometry_counts(&result.center_contacts,&result.corner_contacts);
  ic1_native_distortion_observation(result.distortion_sigma.data(),result.distortion_parameters.data(),&result.distortion_flag);
  if(result.status==0) {
    const auto raw=result.values;
    for(unsigned n=0;n<8;++n) {
      const auto source=history.reference.reference.permutation[n];
      for(unsigned axis=0;axis<3;++axis)result.values[22+3*source+axis]=raw[22+3*n+axis];
      result.values[82+source]=raw[82+n];
    }
  }
  return result;
}
inline void Ready(const Trial& trial) {
  ASSERT_EQ(trial.status,0);ASSERT_EQ(trial.stages,native::RequiredStages);
  for(const auto value:trial.values)ASSERT_TRUE(std::isfinite(value));
  EXPECT_GT(trial.values[6],0.);EXPECT_GT(trial.distortion_parameters[4],0.);
  for(unsigned n=0;n<8;++n)EXPECT_DOUBLE_EQ(trial.values[82+n],.25*trial.values[81]);
}
inline double ForceNorm(const Trial& trial) {
  double sum=0;for(unsigned i=22;i<46;++i)sum+=trial.values[i]*trial.values[i];return std::sqrt(sum);
}
} // namespace solid24_icontrol_test
