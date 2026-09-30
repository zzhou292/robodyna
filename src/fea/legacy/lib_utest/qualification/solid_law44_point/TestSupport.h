#pragma once
#include "lib_src/materials/law44/solid/Update.h"
#include "source_fixture/OriginalCurve.h"
#include <array>
#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>

namespace law44_solid_test {
namespace law = tl::material::law44::solid;
inline law::Material Material(bool bar = false,
    law::WorkingUnits units = law::WorkingUnits::TonneMillimetreSecond) {
  return {(bar ? 50000. : 200000.)*1e6, .3, 7.8900e-9*1e12,
          8000., 8., 10000., units};
}
inline law::Parameters Parameters(bool bar = false,
    law::WorkingUnits units = law::WorkingUnits::TonneMillimetreSecond) {
  law::Parameters p{};
  EXPECT_EQ(law::Prepare(Material(bar, units), {X,Y,Count}, p), law::Status::Ok);
  return p;
}
TL_LAW44_SOLID_HD inline law::Input Motion(unsigned step) {
  law::Input in{};
  in.dt_s = 1e-5;
  const double sign = (step % 80 < 45) ? 1 : -1;
  in.engineering_rate_per_s[0] = sign * 95;
  in.engineering_rate_per_s[1] = -sign * 22;
  in.engineering_rate_per_s[2] = sign * 14;
  in.engineering_rate_per_s[3] = sign * 120;
  in.engineering_rate_per_s[4] = 31 * ::sin(.2*step);
  in.engineering_rate_per_s[5] = -17 * ::cos(.1*step);
  in.relative_density = .002 * ::sin(.05*step);
  return in;
}
inline std::array<double,19> Values(const law::Result& r) {
  std::array<double,19> a{};
  std::copy_n(r.history.stress_pa,6,a.begin());
  std::copy_n(r.history.engineering_strain,6,a.begin()+6);
  a[12]=r.history.plastic_strain;
  a[13]=r.history.filtered_rate_per_s;
  a[14]=r.plastic_increment;
  a[15]=r.yield_stress_pa;
  a[16]=r.sound_speed_m_s;
  a[17]=r.material_viscosity_pa_s;
  a[18]=r.tangent_factor;
  return a;
}
inline void Same(const law::Result& a,const law::Result& b) {
  EXPECT_EQ(Values(a),Values(b));
  EXPECT_EQ(a.history.curve_cursor,b.history.curve_cursor);
}
// Stress cancellation is compared against the actually executed stress-scale
// operations. Native finite SIGY caps are intentionally not a stress-error scale.
inline double StressTolerance(const law::Parameters& p,const law::History& h,
                              const law::Input& in) {
  double scale=std::abs(p.bulk_pa*in.relative_density);
  for(unsigned i=0;i<6;++i) {
    scale+=std::abs(h.stress_pa[i]);
    scale+=p.twice_shear_pa*std::abs(in.engineering_rate_per_s[i]*in.dt_s);
  }
  return 3e-11*std::max(scale,1e-12);
}
}  // namespace law44_solid_test
