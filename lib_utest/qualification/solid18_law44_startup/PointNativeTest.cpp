// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <vector>
extern "C" void law44_solid_initialize_native(int,const double*,const double*,int,
    const double*,double,double*,int*,double*,int*);
extern "C" void law44_solid_native(int,const double*,const double*,int,const double*,int,
    const double*,double,double*,int*,double*,int*);

namespace rear_startup_test {
TEST(Rear18StartupNative, PointTT0UsesCompleteNativeDefaultsFilterAndPressure) {
  namespace p = law::point;
  for (auto units : {p::WorkingUnits::SI,p::WorkingUnits::TonneMillimetreSecond}) {
    const auto parameters = law44_solid_test::Parameters(false,units);
    const auto& m = parameters.material;
    const double material[]{m.young_pa,m.poisson_ratio,m.density_kg_m3,
        m.rate_c_per_s,m.rate_p,0,270e6};
    std::vector<double> curve(2*(parameters.curve.count+1));
    for (unsigned k = 0; k < parameters.curve.count; ++k) {
      curve[2*(k+1)] = parameters.curve.plastic_strain[k];
      curve[2*(k+1)+1] = parameters.curve.yield_stress_pa[k];
    }
    p::Input input;
    const double motion[]{13,-2,9,-8,3,5,0};
    std::copy_n(motion,6,input.engineering_rate_per_s);
    input.relative_density = 1e-4;
    p::Result actual;
    ASSERT_EQ(p::Initialize(parameters,input,actual),p::Status::Ok);
    double expected[19]{}, prepared[26]{}, base[14]{};
    int cursor = -1, status = -1;
    law44_solid_initialize_native(parameters.curve.count,curve.data(),material,int(units),
        motion,input.relative_density,expected,&cursor,prepared,&status);
    ASSERT_EQ(status,0);
    const auto values = law44_solid_test::Values(actual);
    for (unsigned k = 0; k < values.size(); ++k) {
      SCOPED_TRACE(k);
      const double tolerance = k < 6 ? law44_solid_test::StressTolerance(parameters,{},input) :
          3e-11*std::max({std::abs(values[k]),std::abs(expected[k]),1e-12});
      EXPECT_NEAR(values[k],expected[k],tolerance);
    }
    EXPECT_EQ(cursor,0);
    law44_solid_native(parameters.curve.count,curve.data(),material,int(units),base,0,
        motion,input.relative_density,expected,&cursor,prepared,&status);
    EXPECT_NE(status,0); // The old native ordinary ABI still rejects dt0.
  }
}
}  // namespace rear_startup_test
