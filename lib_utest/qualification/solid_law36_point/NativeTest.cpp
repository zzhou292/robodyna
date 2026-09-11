#include "NativeOracle.h"
namespace law36_test {
TEST(SolidLaw36Native, OriginalStaticCurveIndependentSixChannelHistoriesAndSentinels) {
  const auto p=Parameters();
  law::History actual{},native{},zero_defaults{};
  unsigned yielded=0,elastic=0;
  for (unsigned s=0;s<280;++s) {
    SCOPED_TRACE(s);
    const law::Input in{Motion(s),.025*::sin(.04*s)};
    const auto expected=Native(p,native,in);
    const auto zero=Native(p,zero_defaults,in,0);
    law::Result r{};
    ASSERT_EQ(law::Update(p,actual,in,r),law::Status::Ok);
    ASSERT_TRUE(Close(Values(r),expected));
    ASSERT_TRUE(Close(expected,zero));
    native=NativeHistory(expected);
    zero_defaults=NativeHistory(zero);
    actual=r.history;
    yielded+=r.plastic_increment>0;
    elastic+=r.plastic_increment==0;
  }
  EXPECT_GT(yielded,120u);
  EXPECT_GT(elastic,20u);
  EXPECT_GT(actual.plastic_strain,.03);
}
TEST(SolidLaw36Native, HydrostaticIsochoricAndEveryKnotPreserveNativePressureAndYield) {
  const auto p=Parameters();
  for (double knot:X) {
    law::History h{};
    h.plastic_strain=knot;
    for (unsigned channel=0;channel<6;++channel) {
      SCOPED_TRACE(knot);
      SCOPED_TRACE(channel);
      law::Input in{};
      in.kinematics.dt_s=1.0/4096;
      in.kinematics.engineering_rate_per_s[channel]=40;
      in.relative_density=.007;
      law::Result r{};
      ASSERT_EQ(law::Update(p,h,in,r),law::Status::Ok);
      ASSERT_TRUE(Close(Values(r),Native(p,h,in)));
    }
  }
  for (double density:{-.08,0.0,.08}) {
    law::Input in{};
    in.kinematics.dt_s=.001;
    for (unsigned i=0;i<3;++i) in.kinematics.engineering_rate_per_s[i]=3;
    in.relative_density=density;
    law::Result r{};
    ASSERT_EQ(law::Update(p,{},in,r),law::Status::Ok);
    ASSERT_TRUE(Close(Values(r),Native(p,{},in)));
    EXPECT_EQ(r.plastic_increment,0);
    EXPECT_EQ(r.history.deviatoric_rate_per_s,0);
  }
}
TEST(SolidLaw36Native, IndependentCallerWorkUsesActualVolumeDensityAndRoundedPlasticIncrement) {
  const auto p=Parameters();
  law::CallerHistory actual{},native{};
  double max_work=0;
  for (unsigned s=0;s<280;++s) {
    SCOPED_TRACE(s);
    const auto k=Motion(s);
    const auto m=Measures(s);
    const auto expected=Native(p,native,k,m);
    law::CallerResult r{};
    ASSERT_EQ(law::UpdateCaller(p,actual,k,m,r),law::Status::Ok);
    ASSERT_TRUE(Close(Values(r),expected));
    native=NativeHistory(expected);
    actual=r.history;
    max_work=std::max(max_work,std::abs(r.internal_work_j));
    if (s==40) {
      auto wrong=m;
      wrong.density_kg_m3=Rho;
      law::CallerResult control{};
      ASSERT_EQ(law::UpdateCaller(p,actual,k,wrong,control),law::Status::Ok);
      EXPECT_GT(std::abs(control.point.history.stress_pa[0]-r.point.history.stress_pa[0]),1e6);
      wrong=m;
      wrong.new_bulk_pressure_pa+=1e7;
      const auto with_q=Native(p,actual,k,wrong);
      const auto baseline=Native(p,actual,k,m);
      EXPECT_GT(std::abs(with_q[22]-baseline[22]),1e-6);
    }
  }
  EXPECT_GT(actual.plastic_work_j,0);
  EXPECT_GT(max_work,1e-5);
}
TEST(SolidLaw36Native, ZeroDtStartupAndLateOutputFailureRetry) {
  const auto p=Parameters();
  auto k=Motion(0);
  k.dt_s=0;
  const law::Measures m{Rho,2e-8,2e-8,0,0,0};
  law::CallerResult r{};
  ASSERT_EQ(law::UpdateCaller(p,{},k,m,r),law::Status::Ok);
  ASSERT_TRUE(Close(Values(r),Native(p,{},k,m)));
  const auto h=r.history;
  k=Motion(0);
  ASSERT_EQ(law::UpdateCaller(p,h,k,Measures(0),r),law::Status::Ok);
  const auto saved=Bytes(r);
  auto bad=Measures(0);
  bad.storage_volume_m3=std::numeric_limits<double>::max();
  auto with_energy=h;
  with_energy.internal_energy_density_j_m3=2;
  EXPECT_EQ(law::UpdateCaller(p,with_energy,k,bad,r),law::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(r),saved);
  ASSERT_EQ(law::UpdateCaller(p,h,k,Measures(0),r),law::Status::Ok);
  EXPECT_EQ(Bytes(r),saved);
  ASSERT_TRUE(Close(Values(r),Native(p,h,k,Measures(0))));
}
}
