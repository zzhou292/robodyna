#include "WallBoostTestFixture.h"

namespace tl::qualification::qeph::wall_recurrence {
namespace bt=boost_test;
TEST(QephWallScreenSelection, PerStepFactorTwoMarginRetainsDiagnostic4H0Failure) {
  const auto clean=bt::CompleteSet();
  auto jobs=clean.jobs; auto boosts=clean.boosts;
  auto result=SelectWallScreen(jobs,boosts);
  ASSERT_TRUE(result.input_valid)<<result.diagnostic; EXPECT_EQ(result.selected_h,recurrence::H0);
  // Input order is immaterial; every exact tuple must remain represented.
  std::swap(jobs[0],jobs[5]); std::swap(boosts[0],boosts[3]);
  result=SelectWallScreen(jobs,boosts);
  ASSERT_TRUE(result.input_valid); EXPECT_EQ(result.selected_h,recurrence::H0);
  jobs=clean.jobs; boosts=clean.boosts;
  for(auto& job:jobs) job.steps[5].passed=false;
  result=SelectWallScreen(jobs,boosts);
  ASSERT_TRUE(result.input_valid); EXPECT_TRUE(result.passed); EXPECT_EQ(result.selected_h,recurrence::H0);
  EXPECT_FALSE(result.steps[5].passed); EXPECT_TRUE(result.steps[4].passed);
  jobs[5].steps[4].passed=false;
  result=SelectWallScreen(jobs,boosts);
  ASSERT_TRUE(result.input_valid); EXPECT_EQ(result.selected_h,recurrence::H0/2);
  EXPECT_FALSE(result.steps[4].passed); EXPECT_TRUE(result.steps[3].passed);
  jobs[0].steps[3].passed=false;
  result=SelectWallScreen(jobs,boosts);
  ASSERT_TRUE(result.input_valid); EXPECT_FALSE(result.passed); EXPECT_EQ(result.selected_h,0);
}
TEST(QephWallScreenSelection, DuplicateMissingMalformedAndContradictoryTuplesReject) {
  const auto clean=bt::CompleteSet();
  for(unsigned fault=0;fault<8;++fault) {
    SCOPED_TRACE(fault);
    auto jobs=clean.jobs; auto boosts=clean.boosts;
    if(fault==0) jobs[5]=jobs[4];
    if(fault==1) boosts[3]=boosts[2];
    if(fault==2) jobs[0].normal_velocity=-0.0;
    if(fault==3) boosts[3]={};
    if(fault==4) jobs[5].steps[5].h=recurrence::Steps[4];
    if(fault==5) boosts[3].dimension=109;
    if(fault==6) boosts[3].steps[0].amplitudes[0].native_matrix.difference=1;
    if(fault==7) jobs[5].steps[0].amplitudes[0].gains[10].weighted=65;
    const auto result=SelectWallScreen(jobs,boosts);
    EXPECT_FALSE(result.input_valid); EXPECT_FALSE(result.passed); EXPECT_EQ(result.selected_h,0);
    EXPECT_FALSE(result.diagnostic.empty());
  }
}
TEST(QephWallScreenSelection, EitherSignedBoostFailureRemainsAnExplicitRequiredContributor) {
  const auto clean=bt::CompleteSet();
  for(unsigned rejected=0;rejected<4;++rejected) {
    SCOPED_TRACE(rejected);
    auto boosts=clean.boosts;
    auto& step=boosts[rejected].steps[0]; auto& amplitude=step.amplitudes[0];
    amplitude.native_matrix.difference=1; amplitude.native_matrix.passed=false;
    amplitude.passed=false; step.passed=false;
    const auto result=SelectWallScreen(clean.jobs,boosts);
    ASSERT_TRUE(result.input_valid)<<result.diagnostic;
    EXPECT_FALSE(result.passed); EXPECT_EQ(result.selected_h,0); EXPECT_FALSE(result.steps[0].passed);
    for(unsigned b=0;b<4;++b) EXPECT_EQ(result.steps[0].boosts[b],b!=rejected);
    for(bool passed:result.steps[0].jobs) EXPECT_TRUE(passed);
    EXPECT_TRUE(result.steps[1].passed);
  }
}
} // namespace tl::qualification::qeph::wall_recurrence
