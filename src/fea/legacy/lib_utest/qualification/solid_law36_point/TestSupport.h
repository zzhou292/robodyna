#pragma once
#include "source_fixture/OriginalMaterial.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <gtest/gtest.h>
namespace law36_test {
namespace law = tl::material::law36;
inline law::Parameters Parameters() {
  law::Parameters p{};
  EXPECT_EQ(law::Prepare(E, Nu, Rho, Curve, p), law::Status::Ok);
  return p;
}
inline std::array<double,14> Values(const law::History& h) {
  std::array<double,14> v{};
  std::copy_n(h.stress_pa,6,v.begin());
  std::copy_n(h.engineering_strain,6,v.begin()+6);
  v[12]=h.plastic_strain;
  v[13]=h.deviatoric_rate_per_s;
  return v;
}
inline std::array<double,20> Values(const law::Result& r) {
  std::array<double,20> v{};
  const auto h=Values(r.history);
  std::copy(h.begin(),h.end(),v.begin());
  v[14]=r.plastic_increment;
  v[15]=r.yield_stress_pa;
  v[16]=r.equivalent_stress_pa;
  v[17]=r.sound_speed_m_s;
  v[18]=r.material_viscosity_pa_s;
  v[19]=r.plastic_work_density_j_m3;
  return v;
}
inline std::array<double,26> Values(const law::CallerResult& r) {
  std::array<double,26> v{};
  const auto p=Values(r.point);
  std::copy(p.begin(),p.end(),v.begin());
  v[20]=r.history.internal_energy_density_j_m3;
  v[21]=r.history.plastic_work_j;
  v[22]=r.internal_work_j;
  v[23]=r.plastic_work_increment_j;
  v[24]=r.relative_density;
  v[25]=r.average_volume_m3;
  return v;
}
template<class T> inline auto Bytes(const T& v) {
  std::array<unsigned char,sizeof(T)> b{};
  std::memcpy(b.data(),&v,sizeof v);
  return b;
}
inline law::Kinematics Motion(unsigned step) {
  law::Kinematics k{};
  k.dt_s=1.0/65536;
  const double sign=step<80?1.0:step<100?0.0:step<200?-1.0:1.0;
  const double rates[6]={54,-17,-13,26,-14,19};
  for (unsigned i=0;i<6;++i) k.engineering_rate_per_s[i]=sign*rates[i];
  return k;
}
inline law::Measures Measures(unsigned step) {
  constexpr double volume=2e-8;
  const double old_ratio=1+.025*::sin(.029*step);
  const double ratio=1+.025*::sin(.029*(step+1));
  return {Rho/ratio,volume,volume*ratio,volume*(ratio-old_ratio),
          2500*::sin(.041*step),2500*::sin(.041*(step+1))};
}
template<std::size_t N> inline bool Close(const std::array<double,N>& a,
                                         const std::array<double,N>& b) {
  for (std::size_t i=0;i<N;++i) {
    if (!std::isfinite(a[i]) || !std::isfinite(b[i]) ||
        std::abs(a[i]-b[i]) > 3e-11*std::max(1e-18,std::max(std::abs(a[i]),std::abs(b[i])))) {
      ADD_FAILURE() << "channel " << i << ": actual " << a[i] << ", native " << b[i];
      return false;
    }
  }
  return true;
}
} // namespace law36_test
