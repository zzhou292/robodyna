#include "ZeroCFixture.h"
#include <cmath>
#include <limits>

namespace zero_c_test {
TEST(FilteredZeroC, ExplicitFilterAdvancesWithoutStrengtheningAndLegacyOffStaysStrict) {
  auto p = Prepare(false);
  Parameters disabled;
  ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(70e9,.22,2500,{X,Y,3},disabled),Status::Ok);
  History history, off;
  history.filtered_rate_per_s = 23.;
  for (unsigned step=0; step<14; ++step) {
    const auto in = Increment(step);
    Result actual, plain;
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,history,in,actual),Status::Ok);
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(disabled,off,in,plain),Status::Ok);
    const double alpha = std::fmin(1.,p.angular_cutoff_per_s*in.dt);
    EXPECT_DOUBLE_EQ(actual.history.filtered_rate_per_s,
        alpha*in.total_strain_rate_per_s+(1.-alpha)*history.filtered_rate_per_s);
    for (unsigned k=0;k<5;++k) EXPECT_DOUBLE_EQ(actual.history.stress[k],plain.history.stress[k]);
    EXPECT_DOUBLE_EQ(actual.history.plastic_strain,plain.history.plastic_strain);
    EXPECT_DOUBLE_EQ(actual.plastic_increment,plain.plastic_increment);
    EXPECT_DOUBLE_EQ(actual.yield_before_pa,plain.yield_before_pa);
    EXPECT_DOUBLE_EQ(actual.plastic_work_density,plain.plastic_work_density);
    if (!in.element_active) {
      EXPECT_EQ(actual.plastic_increment,0.);
      EXPECT_EQ(actual.elastic_thickness_strain,0.);
      EXPECT_EQ(actual.plastic_thickness_strain,0.);
    }
    history=actual.history;
    off=plain.history;
  }
  Result unchanged;
  unchanged.history.plastic_strain=123.;
  const auto bytes=Bytes(unchanged);
  EXPECT_EQ(mat::UpdateLaw44ShellPlasticity(disabled,history,Increment(0),unchanged),Status::InvalidHistory);
  EXPECT_EQ(Bytes(unchanged),bytes);
}
TEST(FilteredZeroC, InvalidPolicyPreparedStateAndLateInputPublishNothing) {
  for (unsigned fault=0;fault<7;++fault) {
    auto rate=Rate();
    switch (fault) {
      case 0:rate.enabled=false;break;
      case 1:rate.cowper_symonds_c_per_s=8000;break;
      case 2:rate.cowper_symonds_p=0;break;
      case 3:rate.cowper_symonds_p=8;break;
      case 4:rate.cutoff_hz=0;break;
      case 5:rate.policy=static_cast<Policy>(99);break;
      case 6:rate.cutoff_hz=std::numeric_limits<double>::infinity();break;
    }
    Parameters p=Prepare(true);
    const auto bytes=Bytes(p);
    EXPECT_EQ(mat::PrepareLinearLaw44ShellPlasticity(70e9,.22,2500,{30e6,1e9},rate,p),Status::InvalidParameters);
    EXPECT_EQ(Bytes(p),bytes);
  }
  for (unsigned fault=0;fault<4;++fault) {
    auto p=Prepare(true);
    History h;
    h.filtered_rate_per_s=17.;
    auto in=Increment(0);
    Result r;
    r.history.plastic_strain=123.;
    const auto bytes=Bytes(r);
    if (fault==0) p.inverse_rate_c=1./8000;
    if (fault==1) p.inverse_rate_p=.125;
    if (fault==2) in.strain_increment[4]=std::numeric_limits<double>::max();
    if (fault==3) in.total_strain_rate_per_s=std::numeric_limits<double>::quiet_NaN();
    EXPECT_NE(mat::UpdateLaw44ShellPlasticity(p,h,in,r),Status::Ok);
    EXPECT_EQ(Bytes(r),bytes);
    p=Prepare(true);
    ASSERT_EQ(mat::UpdateLaw44ShellPlasticity(p,h,Increment(0),r),Status::Ok);
  }
}
TEST(FilteredZeroC, Nip3KeepsDistinctFilteredHistoriesAndRejectsTheLastPointAtomically) {
  auto p=Prepare(true);
  sec::ShellLayeredJ2History h;
  for (unsigned i=0;i<3;++i) h.point[i].filtered_rate_per_s=10.+i*20.;
  sec::ShellLayeredJ2Input in;
  in.reference_thickness=in.reported_thickness=.003;
  in.transverse_shear_modulus=Increment(0).transverse_shear_modulus;
  in.dt=1.e-7;
  in.strain_curvature_increment[0]=.001;
  in.strain_curvature_increment[5]=.1;
  sec::ShellLayeredJ2Result r;
  ASSERT_EQ(sec::UpdateShellLayeredJ2(p,h,in,r),Status::Ok);
  const auto bytes=Bytes(r);
  const double rate=sec::LayeredJ2TotalStrainRate(in);
  const double alpha=p.angular_cutoff_per_s*in.dt;
  for (unsigned i=0;i<3;++i)
    EXPECT_DOUBLE_EQ(r.history.point[i].filtered_rate_per_s,alpha*rate+(1.-alpha)*h.point[i].filtered_rate_per_s);
  h.point[2].filtered_rate_per_s=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(sec::UpdateShellLayeredJ2(p,h,in,r),Status::InvalidHistory);
  EXPECT_EQ(Bytes(r),bytes);
  h.point[2].filtered_rate_per_s=50.;
  ASSERT_EQ(sec::UpdateShellLayeredJ2(p,h,in,r),Status::Ok);
}
} // namespace zero_c_test
