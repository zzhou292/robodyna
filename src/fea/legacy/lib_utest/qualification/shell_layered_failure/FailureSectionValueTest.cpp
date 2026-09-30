#include "FailureSectionFixture.h"
#include <gtest/gtest.h>
#include <limits>
#include <array>
#include <cstring>

namespace layered_failure_test {
TEST(LayeredFailureValues, ExactWeightsAndEveryFailedSubset) {
  for(unsigned mask=0;mask<8;++mask) {
    const auto h=Seed(mask);
    const double expected=((mask&1)?.25:0)+((mask&2)?.5:0)+((mask&4)?.25:0);
    EXPECT_EQ(sec::ShellNip3FailedThickness(h.failure),expected);
    EXPECT_EQ(expected>=sec::ShellNip3FailureThreshold(),mask==7);
  }
}

TEST(LayeredFailureValues, FailedPointStillProducesCurrentStressAndRate) {
  const auto p=Parameters(false,true);
  auto in=Input(p);in.strain_curvature_increment[0]=.005;
  auto base=Seed(1);
  sec::ShellLayeredJ2FailureResult result;
  ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.},base,in,2.e-6,result),sec::PointStatus::Ok);
  EXPECT_TRUE(result.history.element_active);
  EXPECT_GT(result.current.history.point[0].stress[0],0);
  EXPECT_GT(result.history.saved.point[0].plastic_strain,0);
  EXPECT_GT(result.history.saved.point[0].filtered_rate_per_s,0);
  EXPECT_EQ(result.history.saved.point[0].stress[0],0);
  EXPECT_EQ(result.history.failure[0].failure_time_s,1.e-6);
  EXPECT_FALSE(sec::MatchesLayeredJ2Resultants(result.history.saved,result.current));
  EXPECT_TRUE(sec::MatchesLayeredJ2FailureResultants(result.history,result.current));
}

TEST(LayeredFailureValues, RemovalMasksViscosityAndKeepsOldForceWork) {
  const auto p=Parameters();auto in=Input(p);
  in.strain_curvature_increment[0]=.006;
  WorkHistory work;work.stress[0]=120.;work.internal_work[0]=7.;
  sec::ShellLayeredJ2FailureResult result;
  ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.e-5},{},in,1.e-6,result),sec::PointStatus::Ok);
  ASSERT_TRUE(result.removed_now);
  ASSERT_TRUE(sec::ApplyLayeredJ2FailureWork(result,in.strain_curvature_increment,.002,.01,25.,work));
  EXPECT_EQ(work.stress[0],0);
  EXPECT_EQ(work.internal_work[0],7.+120.*.006*(.5*.002*.01));
  EXPECT_GT(result.current.diagnostics.plastic_work_density_increment,0);
  const auto removed=result.history;
  for(unsigned step=2;step<4;++step) {
    ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.e-5},result.history,in,step*1.e-6,result),sec::PointStatus::Ok);
    EXPECT_FALSE(result.removed_now);
    EXPECT_FALSE(result.history.element_active);
    EXPECT_EQ(result.current.reported_thickness,in.reported_thickness);
    EXPECT_EQ(result.current.diagnostics.plastic_work_density_increment,0);
    for(unsigned point=0;point<3;++point) {
      EXPECT_EQ(result.history.saved.point[point].plastic_strain,removed.saved.point[point].plastic_strain);
      EXPECT_EQ(result.history.failure[point].failure_time_s,1.e-6);
    }
  }
}

TEST(LayeredFailureValues, DefaultActivePointAndSectionPreserveLegacyValues) {
  const auto p=Parameters(true,true);auto in=Input(p);
  in.strain_curvature_increment[0]=.004;in.strain_curvature_increment[5]=1.2;
  sec::ShellLayeredJ2Result old;
  sec::ShellLayeredJ2FailureResult current;
  ASSERT_EQ(sec::UpdateShellLayeredJ2(p,{},in,old),sec::PointStatus::Ok);
  ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{2.},{},in,1.e-6,current),sec::PointStatus::Ok);
  for(unsigned point=0;point<3;++point) {
    for(unsigned c=0;c<5;++c) EXPECT_EQ(old.history.point[point].stress[c],current.history.saved.point[point].stress[c]);
    EXPECT_EQ(old.history.point[point].plastic_strain,current.history.saved.point[point].plastic_strain);
    EXPECT_EQ(old.history.point[point].filtered_rate_per_s,current.history.saved.point[point].filtered_rate_per_s);
  }
  for(unsigned c=0;c<5;++c) EXPECT_EQ(old.material_stress[c],current.current.material_stress[c]);
  for(unsigned c=0;c<3;++c) EXPECT_EQ(old.bending_stress[c],current.current.bending_stress[c]);
  EXPECT_EQ(old.reported_thickness,current.current.reported_thickness);
  EXPECT_EQ(old.diagnostics.plastic_work_density_increment,current.current.diagnostics.plastic_work_density_increment);
}

TEST(LayeredFailureValues, LastPointFailureAndInvalidWorkPreserveOutput) {
  const auto p=Parameters();auto in=Input(p);
  sec::ShellLayeredJ2FailureHistory base;
  base.saved.point[2].plastic_strain=3.; // Beyond the retained curve domain.
  sec::ShellLayeredJ2FailureResult result;result.constitutive_increment[0]=73;
  // Snapshot the same object's representation; no comparison of padding in
  // separately evaluated results or ordinary aggregate copies is assumed.
  std::array<unsigned char,sizeof(result)> before{};
  std::memcpy(before.data(),&result,sizeof(result));
  EXPECT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.},base,in,1.e-6,result),sec::PointStatus::CurveDomainExceeded);
  EXPECT_EQ(result.constitutive_increment[0],73);
  EXPECT_EQ(std::memcmp(before.data(),&result,sizeof(result)),0);
  base={};base.failure[2].failure_time_s=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.},base,in,1.e-6,result),sec::PointStatus::InvalidHistory);
  EXPECT_EQ(result.constitutive_increment[0],73);
  EXPECT_EQ(std::memcmp(before.data(),&result,sizeof(result)),0);
  ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{1.},{},in,1.e-6,result),sec::PointStatus::Ok);
  WorkHistory work;work.internal_work[0]=19.;
  result.current.reported_thickness=std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(sec::ApplyLayeredJ2FailureWork(result,in.strain_curvature_increment,.002,.01,0.,work));
  EXPECT_EQ(work.internal_work[0],19.);EXPECT_EQ(work.thickness,.002);
}

TEST(LayeredFailureValues, RoundedCallerWorkAndDamageExcludeSubUlpIncrement) {
  sec::PointParameters p;
  const double x[]{0,1.e12+1},y[]{220e6,220e6};
  ASSERT_EQ(mat::PrepareTabulatedShellPlasticity(200e9,.3,7890,{x,y,2},p),sec::PointStatus::Ok);
  auto in=Input(p);in.strain_curvature_increment[0]=.001101;
  in.strain_curvature_increment[1]=-.3*in.strain_curvature_increment[0];
  sec::ShellLayeredJ2FailureHistory base;
  for(auto& point:base.saved.point) point.plastic_strain=1.e12;
  sec::ShellLayeredJ2FailureResult result;
  ASSERT_EQ(sec::UpdateShellLayeredJ2Failure(p,{.01},base,in,1.e-6,result),sec::PointStatus::Ok);
  for(unsigned point=0;point<3;++point) {
    EXPECT_GT(result.constitutive_increment[point],0);
    EXPECT_EQ(result.caller_failure_increment[point],0);
    EXPECT_EQ(result.history.failure[point].damage,0);
  }
  EXPECT_EQ(result.current.diagnostics.plastic_work_density_increment,0);
}
} // namespace layered_failure_test
