#include "NativeOracle.h"

namespace law44_solid_test {
TEST(SolidLaw44Native, OriginalBlankDefaultsCaZeroAndReturnedSolidSoundSpeed) {
  for(bool bar:{false,true}) {
    const auto p=Parameters(bar);
    law::Input in{};
    in.dt_s=1e-6;
    in.relative_density=.02;
    const auto n=Native(p,{},in);
    EXPECT_EQ(n.prepared[2],0);  // CA cleared, original SIGY270 is not added.
    EXPECT_EQ(n.prepared[3],static_cast<double>(1e20f));
    EXPECT_EQ(n.prepared[4],static_cast<double>(1e20f));
    EXPECT_EQ(n.prepared[10],1./8000);
    EXPECT_EQ(n.prepared[11],1./8);
    EXPECT_EQ(n.prepared[12],1);
    EXPECT_EQ(n.prepared[13],10000);
    EXPECT_EQ(n.prepared[22],2);
    EXPECT_EQ(n.prepared[23],1);
    EXPECT_EQ(n.prepared[25],1);  // Actual reader CA warning observed once.
    law::Result r{};
    ASSERT_EQ(law::Update(p,{},in,r),law::Status::Ok);
    Compare(r,n.result,p,{},in);
    EXPECT_EQ(r.tangent_factor,1);
  }
}
TEST(SolidLaw44Native, IndependentHistoriesOriginalCurveBothModuliAndUnits) {
  for(bool bar:{false,true}) for(auto units:{law::WorkingUnits::SI,law::WorkingUnits::TonneMillimetreSecond}) {
    const auto p=Parameters(bar,units);
    law::History h{},native{};
    unsigned plastic=0,elastic=0;
    for(unsigned step=0;step<320;++step) {
      SCOPED_TRACE(step);
      const auto in=Motion(step);
      law::Result r{};
      ASSERT_EQ(law::Update(p,h,in,r),law::Status::Ok);
      const auto n=Native(p,native,in);
      Compare(r,n.result,p,h,in);
      plastic+=r.plastic_increment>0;
      elastic+=r.plastic_increment==0;
      h=r.history;
      native=n.result.history;
    }
    EXPECT_GT(plastic,100u);
    EXPECT_GT(elastic,5u);
  }
}
TEST(SolidLaw44Native, ExactKnotIncomingSegmentsAndLastExtrapolation) {
  const auto p=Parameters();
  for(unsigned cursor:{0u,1u,44u}) {
    law::History h{};
    h.curve_cursor=cursor;
    h.plastic_strain=cursor==44 ? .35 : X[1];
    h.filtered_rate_per_s=12;
    law::Result r{};
    auto in=Motion(0);
    ASSERT_EQ(law::Update(p,h,in,r),law::Status::Ok);
    Compare(r,Native(p,h,in).result,p,h,in);
    EXPECT_EQ(r.history.curve_cursor,cursor);
  }
}
TEST(SolidLaw44Native, NativeFiniteLimitsAndUnitConvertedStressFloors) {
  auto p=Parameters();
  for(unsigned mode=0;mode<3;++mode) {
    law::History h{};
    if(mode==0) h.plastic_strain=law::detail::NativeInfinity();
    if(mode==1) h.engineering_strain[0]=1.5*law::detail::NativeInfinity();
    if(mode==2) {
      h.plastic_strain=.01;
      h.engineering_strain[0]=3*law::detail::NativeInfinity();
    }
    law::Result r{};
    ASSERT_EQ(law::Update(p,h,Motion(0),r),law::Status::Ok);
    Compare(r,Native(p,h,Motion(0)).result,p,h,Motion(0));
  }
  const double x[]={0,1},y[]={1e-18,2e-18};
  law::Result results[2]{};
  for(unsigned mode=0;mode<2;++mode) {
    auto m=Material();
    m.young_pa=1e-14;
    m.native_units=mode ? law::WorkingUnits::TonneMillimetreSecond : law::WorkingUnits::SI;
    ASSERT_EQ(law::Prepare(m,{x,y,2},p),law::Status::Ok);
    auto in=Motion(0);
    in.dt_s=.001;
    ASSERT_EQ(law::Update(p,{},in,results[mode]),law::Status::Ok);
    Compare(results[mode],Native(p,{},in).result,p,{},in);
  }
  EXPECT_NE(results[0].plastic_increment,results[1].plastic_increment);
}
}  // namespace law44_solid_test
