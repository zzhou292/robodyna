#include "TestSupport.h"
#include <limits>
namespace law36_test {
TEST(SolidLaw36, OriginalMaterialAndThreeDimensionalHydrostaticResponse) {
  const auto p=Parameters();
  EXPECT_DOUBLE_EQ(p.density_kg_m3,1070);
  EXPECT_DOUBLE_EQ(p.young_pa,1887000000);
  EXPECT_NE(Y[0],10.041*1e6); // Table overrides the supplied SIGY card.
  EXPECT_GT(p.bulk_pa,p.young_pa);
  law::Input in{};
  in.kinematics.dt_s=.01;
  in.relative_density=.125;
  law::Result r{};
  ASSERT_EQ(law::Update(p,{},in,r),law::Status::Ok);
  for (unsigned i=0;i<3;++i) EXPECT_DOUBLE_EQ(r.history.stress_pa[i],-p.bulk_pa*.125);
  EXPECT_EQ(r.plastic_increment,0);
  EXPECT_EQ(r.equivalent_stress_pa,0);
  EXPECT_EQ(r.history.deviatoric_rate_per_s,0);
  in.kinematics.engineering_rate_per_s[2]=.5;
  ASSERT_EQ(law::Update(p,{},in,r),law::Status::Ok);
  EXPECT_NE(r.history.stress_pa[2],r.history.stress_pa[1]);
  EXPECT_NE(r.history.engineering_strain[2],0);
}
TEST(SolidLaw36, AllChannelsYieldUnloadReloadAndStaticRateBranch) {
  const auto p=Parameters();
  law::History h{};
  unsigned yielded=0, held=0;
  for (unsigned s=0;s<280;++s) {
    law::Input in{Motion(s),.02*::sin(.02*s)};
    law::Result r{},scaled{};
    ASSERT_EQ(law::Update(p,h,in,r),law::Status::Ok);
    auto twice=in;
    twice.kinematics.dt_s*=2;
    for (double& v:twice.kinematics.engineering_rate_per_s) v*=.5;
    ASSERT_EQ(law::Update(p,h,twice,scaled),law::Status::Ok);
    for (unsigned i=0;i<6;++i) EXPECT_DOUBLE_EQ(r.history.stress_pa[i],scaled.history.stress_pa[i]);
    EXPECT_DOUBLE_EQ(r.history.plastic_strain,scaled.history.plastic_strain);
    EXPECT_DOUBLE_EQ(r.history.deviatoric_rate_per_s*.5,scaled.history.deviatoric_rate_per_s);
    yielded+=r.plastic_increment>0;
    held+=r.plastic_increment==0;
    h=r.history;
  }
  EXPECT_GT(yielded,120u);
  EXPECT_GT(held,20u);
  EXPECT_GT(h.plastic_strain,.03);
  for (double s:h.stress_pa) EXPECT_NE(s,0);
}
TEST(SolidLaw36, CallerPreservesPressureVolumeAndSignedInternalWork) {
  const auto p=Parameters();
  law::CallerHistory h{};
  auto k=Motion(10);
  k.engineering_rate_per_s[0]=k.engineering_rate_per_s[1]=k.engineering_rate_per_s[2]=0;
  k.engineering_rate_per_s[3]=k.engineering_rate_per_s[4]=k.engineering_rate_per_s[5]=0;
  const law::Measures m{Rho*1.03,2e-8,1.9e-8,-1e-9,3000,5000};
  law::CallerResult r{};
  ASSERT_EQ(law::UpdateCaller(p,h,k,m,r),law::Status::Ok);
  const double pressure=p.bulk_pa*r.relative_density;
  const double expected=-m.volume_increment_m3*(8000+pressure)*.5;
  EXPECT_NEAR(r.internal_work_j,expected,2e-15*std::abs(expected));
  EXPECT_DOUBLE_EQ(r.average_volume_m3,m.current_volume_m3-.5*m.volume_increment_m3);
  EXPECT_EQ(r.plastic_work_increment_j,0);
  EXPECT_GT(r.internal_work_j,0);
}
TEST(SolidLaw36, LateInvalidCurveOverflowAndRetryPreserveAcceptedValues) {
  auto p=Parameters();
  const auto saved_p=Bytes(p);
  double y[8];
  std::copy_n(Y,8,y);
  y[7]=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(law::Prepare(E,Nu,Rho,{X,y,8},p),law::Status::InvalidCurve);
  EXPECT_EQ(Bytes(p),saved_p);
  EXPECT_EQ(law::Prepare(E,Nu,Rho,{X,Y,1025},p),law::Status::InvalidCurve);
  EXPECT_EQ(Bytes(p),saved_p);
  law::CallerHistory h{};
  law::CallerResult r{};
  auto k=Motion(0);
  const auto m=Measures(0);
  ASSERT_EQ(law::UpdateCaller(p,h,k,m,r),law::Status::Ok);
  const auto saved=Bytes(r);
  const auto base=Bytes(h);
  k.engineering_rate_per_s[5]=std::numeric_limits<double>::max();
  EXPECT_NE(law::UpdateCaller(p,h,k,m,r),law::Status::Ok);
  EXPECT_EQ(Bytes(r),saved);
  EXPECT_EQ(Bytes(h),base);
  auto bad=m;
  bad.new_bulk_pressure_pa=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(law::UpdateCaller(p,h,Motion(0),bad,r),law::Status::InvalidInput);
  EXPECT_EQ(Bytes(r),saved);
  ASSERT_EQ(law::UpdateCaller(p,h,Motion(0),m,r),law::Status::Ok);
  EXPECT_EQ(Bytes(r),saved);
  h.point.plastic_strain=law::detail::NativeSentinel();
  EXPECT_EQ(law::UpdateCaller(p,h,Motion(0),m,r),law::Status::SentinelDomainExceeded);
  EXPECT_EQ(Bytes(r),saved);
}
TEST(SolidLaw36, ZeroDtIsVirginStartupOnly) {
  const auto p=Parameters();
  auto k=Motion(0);
  k.dt_s=0;
  const law::Measures m{Rho,2e-8,2e-8,0,0,0};
  law::CallerResult r{};
  ASSERT_EQ(law::UpdateCaller(p,{},k,m,r),law::Status::Ok);
  EXPECT_EQ(r.internal_work_j,0);
  EXPECT_EQ(r.point.plastic_increment,0);
  EXPECT_GT(r.point.history.deviatoric_rate_per_s,0);
  const auto saved=Bytes(r);
  EXPECT_EQ(law::UpdateCaller(p,r.history,k,m,r),law::Status::InvalidInput);
  EXPECT_EQ(Bytes(r),saved);
}
} // namespace law36_test
