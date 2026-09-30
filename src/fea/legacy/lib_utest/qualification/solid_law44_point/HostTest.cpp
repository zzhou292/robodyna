#include "TestSupport.h"
#include <limits>

namespace law44_solid_test {
TEST(SolidLaw44, OriginalModuliAndNativeUnitSentinelsAreDistinct) {
  const auto tube=Parameters(),bar=Parameters(true),si=Parameters(false,law::WorkingUnits::SI);
  EXPECT_EQ(tube.material.density_kg_m3,7.8900e-9*1e12);
  EXPECT_EQ(tube.material.young_pa,4*bar.material.young_pa);
  EXPECT_EQ(tube.stress_limit_pa,static_cast<double>(1e20f)*1e6);
  EXPECT_EQ(tube.stress_floor_pa,1e-20*1e6);
  EXPECT_EQ(si.stress_limit_pa,static_cast<double>(1e20f));
  EXPECT_EQ(tube.shear_pa,si.shear_pa);
  EXPECT_NE(tube.sound_speed_m_s,std::sqrt(tube.material.young_pa /
      (1-.3*.3)/tube.material.density_kg_m3));
  EXPECT_EQ(Count,46u);
  EXPECT_EQ(Y[0],270e6);
  EXPECT_EQ(X[45],.3);
}
TEST(SolidLaw44, PressureUsesSuppliedAmuAndFilterUsesFullTensor) {
  const auto p=Parameters();
  law::Input in{};
  in.dt_s=1e-6;
  in.relative_density=.125;
  law::Result result{};
  ASSERT_EQ(law::Update(p,{},in,result),law::Status::Ok);
  for(unsigned i=0;i<3;++i) EXPECT_EQ(result.history.stress_pa[i],-p.bulk_pa*.125);
  EXPECT_EQ(result.plastic_increment,0);
  EXPECT_EQ(result.tangent_factor,1);
  EXPECT_EQ(result.material_viscosity_pa_s,0);
  in.relative_density=0;
  in.engineering_rate_per_s[0]=in.engineering_rate_per_s[1]=in.engineering_rate_per_s[2]=2;
  ASSERT_EQ(law::Update(p,{},in,result),law::Status::Ok);
  const double alpha=p.angular_cutoff_per_s*in.dt_s;
  EXPECT_EQ(result.history.filtered_rate_per_s,alpha*std::sqrt(12.));
  EXPECT_GT(result.history.filtered_rate_per_s,0);  // Hydrostatic rate is not discarded.
  for(unsigned i=0;i<3;++i) EXPECT_EQ(result.history.engineering_strain[i],2*in.dt_s);
  law::Input rest{};
  rest.dt_s=in.dt_s;
  law::Result unloaded{};
  ASSERT_EQ(law::Update(p,result.history,rest,unloaded),law::Status::Ok);
  EXPECT_EQ(unloaded.history.filtered_rate_per_s,(1-alpha)*result.history.filtered_rate_per_s);
}
TEST(SolidLaw44, InitialYoungHardeningAndLaterCurveSlopePreserveEt) {
  const auto p=Parameters();
  law::Input in{};
  in.dt_s=1e-3;
  in.engineering_rate_per_s[3]=20;
  law::Result first{},second{};
  ASSERT_EQ(law::Update(p,{},in,first),law::Status::Ok);
  ASSERT_GT(first.plastic_increment,0);
  EXPECT_EQ(first.tangent_factor,.5);  // Native PLA0 uses H=E, not first curve slope.
  EXPECT_GT(first.yield_stress_pa,Y[0]);
  ASSERT_EQ(law::Update(p,first.history,in,second),law::Status::Ok);
  ASSERT_GT(second.plastic_increment,0);
  EXPECT_LT(second.tangent_factor,.1);
  EXPECT_GT(second.history.plastic_strain,first.history.plastic_strain);
  EXPECT_GE(second.history.curve_cursor,first.history.curve_cursor);
  unsigned plastic=0,elastic=0;
  auto history=second.history;
  for(unsigned step=0;step<240;++step) {
    law::Result r{};
    ASSERT_EQ(law::Update(p,history,Motion(step),r),law::Status::Ok);
    plastic+=r.plastic_increment>0;
    elastic+=r.plastic_increment==0;
    if(r.plastic_increment==0) EXPECT_EQ(r.tangent_factor,1);
    history=r.history;
  }
  EXPECT_GT(plastic,100u);
  EXPECT_GT(elastic,5u);
}
TEST(SolidLaw44, ForwardCursorRetainsKnotAndExtrapolatesLastSegment) {
  const double x[]={0,.1,.2},y[]={1,2,4};
  const law::Curve c{x,y,3};
  std::uint32_t cursor=0;
  double value=0,slope=0;
  ASSERT_TRUE(law::detail::CurveValue(c,.1,cursor,value,slope));
  EXPECT_EQ(cursor,0u);
  EXPECT_EQ(slope,10);
  ASSERT_TRUE(law::detail::CurveValue(c,std::nextafter(.1,1.),cursor,value,slope));
  EXPECT_EQ(cursor,1u);
  EXPECT_EQ(slope,20);
  ASSERT_TRUE(law::detail::CurveValue(c,.1,cursor,value,slope));
  EXPECT_EQ(cursor,1u);
  EXPECT_EQ(slope,20);  // Forward-only VINTER does not become VINTER2.
  ASSERT_TRUE(law::detail::CurveValue(c,.4,cursor,value,slope));
  EXPECT_NEAR(value,8,1e-14);
  cursor=2;
  EXPECT_FALSE(law::detail::CurveValue(c,.4,cursor,value,slope));
  EXPECT_EQ(cursor,2u);
}
TEST(SolidLaw44, InvalidLateInputAndParametersLeaveOutputAndHistoryUnchanged) {
  auto p=Parameters();
  law::Result result{};
  ASSERT_EQ(law::Update(p,{},Motion(0),result),law::Status::Ok);
  const auto saved=result;
  auto input=Motion(0);
  input.engineering_rate_per_s[5]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(law::Update(p,{},input,result),law::Status::InvalidInput);
  Same(result,saved);
  input.engineering_rate_per_s[5]=std::numeric_limits<double>::max();
  EXPECT_EQ(law::Update(p,{},input,result),law::Status::NonfiniteResult);
  Same(result,saved);
  input=Motion(0);
  input.dt_s=0;
  EXPECT_EQ(law::Update(p,{},input,result),law::Status::InvalidInput);
  Same(result,saved);
  auto history=saved.history;
  history.curve_cursor=Count-1;
  EXPECT_EQ(law::Update(p,history,Motion(0),result),law::Status::InvalidHistory);
  Same(result,saved);
  EXPECT_EQ(history.curve_cursor,Count-1);
  p.inverse_rate_p=1;
  EXPECT_EQ(law::Update(p,{},Motion(0),result),law::Status::InvalidParameters);
  Same(result,saved);
  ASSERT_EQ(law::Update(Parameters(),{},Motion(0),result),law::Status::Ok);
  Same(result,saved);
}
TEST(SolidLaw44, LateCurveRejectionAndUnknownUnitsPreservePreparedValues) {
  auto p=Parameters();
  const auto saved=p;
  double y[Count];
  std::copy_n(Y,Count,y);
  y[Count-1]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(law::Prepare(Material(),{X,y,Count},p),law::Status::InvalidCurve);
  EXPECT_EQ(p.curve.yield_stress_pa,saved.curve.yield_stress_pa);
  EXPECT_EQ(p.sound_speed_m_s,saved.sound_speed_m_s);
  auto m=Material();
  m.native_units=static_cast<law::WorkingUnits>(99);
  EXPECT_EQ(law::Prepare(m,{X,Y,Count},p),law::Status::InvalidParameters);
  EXPECT_EQ(p.stress_limit_pa,saved.stress_limit_pa);
  EXPECT_EQ(law::Prepare(Material(),{X,Y,1025},p),law::Status::InvalidCurve);
  ASSERT_EQ(law::Prepare(Material(),{X,Y,Count},p),law::Status::Ok);
  EXPECT_EQ(p.shear_pa,saved.shear_pa);
}
TEST(SolidLaw44, StressComparisonDoesNotHideUnitOrFiniteCapCorruption) {
  const auto p=Parameters();
  law::History h{};
  h.plastic_strain=law::detail::NativeInfinity();
  law::Result result{};
  const auto in=Motion(0);
  ASSERT_EQ(law::Update(p,h,in,result),law::Status::Ok);
  ASSERT_GT(result.yield_stress_pa,1e25);
  const double tolerance=StressTolerance(p,h,in);
  EXPECT_LT(tolerance,.1);
  const double corrupted=result.history.stress_pa[0]+1.;
  EXPECT_GT(std::abs(corrupted-result.history.stress_pa[0]),tolerance);
  EXPECT_GT(std::abs(p.sound_speed_m_s*1000-p.sound_speed_m_s),
            3e-11*p.sound_speed_m_s*1000);
}
}  // namespace law44_solid_test
