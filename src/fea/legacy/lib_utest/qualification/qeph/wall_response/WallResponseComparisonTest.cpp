#include "WallResponseComparison.h"
#include "WallResponseComparisonBounds.h"
#include "WallResponseComparisonTestFixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

namespace tl::qualification::qeph::wall_response::comparison_test {
TEST(QephWallResponseComparison, IntervalDifferencesRetainCertificateWidthsAndStageFailures) {
  namespace b=comparison_detail;
  Interval result;
  ASSERT_TRUE(b::Difference(.75,.125,.25,.0625,1,result));
  EXPECT_EQ(result.lower,.3125); EXPECT_EQ(result.upper,.6875);
  ASSERT_TRUE(b::Difference(1,.125,1,.125,1,result));
  EXPECT_EQ(result.lower,0); EXPECT_EQ(result.upper,.25);
  ASSERT_TRUE(b::Difference(1,0,1,0,wr::TargetDepth,result));
  EXPECT_EQ(result.lower,0); EXPECT_EQ(result.upper,0);
  result={.25,.5};
  EXPECT_FALSE(b::Difference(1,-1,1,0,1,result));
  EXPECT_EQ(result.lower,.25); EXPECT_EQ(result.upper,.5);
  EXPECT_FALSE(b::Difference(std::numeric_limits<double>::max(),0,
      -std::numeric_limits<double>::max(),0,1,result));
  EXPECT_EQ(result.lower,.25); EXPECT_EQ(result.upper,.5);
  EXPECT_FALSE(b::Difference(1,0,0,0,0,result));
  EXPECT_EQ(result.lower,.25); EXPECT_EQ(result.upper,.5);
}
TEST(QephWallResponseComparison, DirectedThresholdAndCoarseLowerBoundCannotRoundIntoPassing) {
  namespace b=comparison_detail;
  const double denominator=std::nextafter(std::nextafter(1.,0.),0.);
  const double numerator=CoarseResponseLimit*denominator;
  ASSERT_EQ(numerator/denominator,CoarseResponseLimit);
  ASSERT_GT(static_cast<long double>(numerator)/denominator,static_cast<long double>(CoarseResponseLimit));
  Interval normalized;
  ASSERT_TRUE(b::Normalize({numerator,numerator},denominator,normalized));
  EXPECT_GT(normalized.upper,CoarseResponseLimit);
  EXPECT_FALSE(b::Refines({.0061,.0061},{.008,.01},ResponseContractionFloor));
  EXPECT_TRUE(b::Refines({0,0},{0,0},ResponseContractionFloor));
  double rounded=.75*.02+ResponseContractionFloor;
  const long double truth=.75L*static_cast<long double>(.02)+ResponseContractionFloor;
  if(static_cast<long double>(rounded)<=truth) rounded=std::nextafter(rounded,INFINITY);
  ASSERT_GT(static_cast<long double>(rounded),truth);
  EXPECT_FALSE(b::Refines({rounded,rounded},{.02,.02},ResponseContractionFloor));
  EXPECT_FALSE(b::Refines({0,std::numeric_limits<double>::infinity()},{.02,.02},ResponseContractionFloor));
}
TEST(QephWallResponseComparison, PhysicalPotentialDifferencesAndInitialEnergyUseFixedScales) {
  const auto runs=Sequence(); const auto result=Compare(runs);
  ASSERT_TRUE(result.input_valid)<<result.diagnostic; ASSERT_TRUE(result.passed)<<result.diagnostic;
  EXPECT_EQ(result.energy_normalization,runs[0].model.energy());
  EXPECT_EQ(result.screen_index_sha,runs[0].config.screen_index_sha);
  EXPECT_EQ(runs[0].model.dictionary()[result.coarse_medium.field].name,"contact.potential");
  long double expected=0; unsigned maximum_sample=0;
  // Independent quadratic potential calculation, using the recorded raw
  // positions and immutable point stiffness. No comparison/observer formula.
  for(unsigned j=0;j<SampleCount;++j) {
    long double difference=0;
    for(unsigned n=0;n<runs[0].model.fields().nodes;++n) {
      const long double a=std::max(0.,runs[0].samples[j].state.x[3*n]);
      const long double b=std::max(0.,runs[1].samples[j].state.x[3*n]);
      difference+=.5L*runs[0].model.screened().touching().nodes[n].stiffness.value*(a*a-b*b);
    }
    difference/=result.energy_normalization;
    if(difference>expected) { expected=difference; maximum_sample=j; }
  }
  EXPECT_NEAR(result.coarse_medium.maximum.upper,static_cast<double>(expected),2e-10);
  EXPECT_EQ(result.coarse_medium.time,runs[0].samples[maximum_sample].time);
  EXPECT_GT(result.coarse_medium.maximum.lower,.008); EXPECT_LT(result.coarse_medium.maximum.upper,.0081);
  EXPECT_GT(result.medium_fine.maximum.lower,.004); EXPECT_LT(result.medium_fine.maximum.upper,.0041);
  EXPECT_GT(result.residual_ratios[0].lower,.016); EXPECT_LT(result.residual_ratios[0].upper,.0161);
  EXPECT_TRUE(result.response_passed); EXPECT_TRUE(result.energy_passed); EXPECT_TRUE(result.analytic_passed);
}
TEST(QephWallResponseComparison, NoncontractingResponseAndEnergyAreRejectedWithoutChangingScales) {
  auto runs=Sequence(); runs[2]=Synthetic(4,1,.002);
  const auto response=Compare(runs);
  ASSERT_TRUE(response.input_valid)<<response.diagnostic;
  EXPECT_FALSE(response.passed); EXPECT_FALSE(response.response_passed);
  EXPECT_LT(response.coarse_medium.maximum.upper,CoarseResponseLimit);
  EXPECT_LT(response.medium_fine.maximum.upper,FineResponseLimit);
  EXPECT_TRUE(response.energy_passed);
  runs=Sequence();
  // An all-endpoint extremum can exceed every retained common sample. It is
  // authenticated producer evidence, not reconstructed unsaved native work.
  runs[2].summary.maximum_absolute_residual=runs[1].summary.maximum_absolute_residual;
  const auto energy=Compare(runs); ASSERT_TRUE(energy.input_valid)<<energy.diagnostic;
  EXPECT_FALSE(energy.passed); EXPECT_FALSE(energy.energy_passed); EXPECT_TRUE(energy.response_passed);
}
TEST(QephWallResponseComparison, ImmutableIdentitySamplePhasesAndFinalAssociationRemainRequired) {
  auto runs=Sequence(); const auto authentic=runs[2]; std::string error;
  runs[2].config.screen_index_sha=std::string(64,'b');
  EXPECT_FALSE(Compare(runs).input_valid);
  runs[2]=authentic; runs[2].samples[5].carried_velocity_time+=H0;
  EXPECT_FALSE(ValidateRun(runs[2],true,error));
  runs[2]=authentic; runs[2].samples[5].state.x[0]+=1e-7;
  EXPECT_FALSE(ValidateRun(runs[2],true,error));
  runs[2]=authentic; runs[2].summary.maximum_absolute_residual={0,0};
  EXPECT_FALSE(ValidateRun(runs[2],true,error));
  runs[2]=authentic; runs[2].last_accepted.state.q[0]=-1;
  runs[2].last_accepted.values[runs[2].model.node_fields(0).q]=-1;
  ASSERT_TRUE(ValidateSample(runs[2].model,runs[2].config,runs[2].last_accepted,error))<<error;
  EXPECT_FALSE(ValidateRun(runs[2],true,error)); // Equivalent rotation, different retained final quaternion.
  runs[2]=authentic; ASSERT_TRUE(ValidateRun(runs[2],true,error))<<error;
  EXPECT_TRUE(Compare(runs).passed);
}
TEST(QephWallResponseComparison, PartialRetryCountsRemainEvidenceAndCannotAdmitACompleteRun) {
  auto partial=Synthetic(1,2,.008,198); std::string error;
  ASSERT_FALSE(partial.completed); ASSERT_TRUE(ValidateRun(partial,false,error))<<error;
  ++partial.attempted_steps; partial.native_cell_intervals+=partial.config.cells;
  partial.failure="Synthetic inspected and discarded final proposal";
  ASSERT_TRUE(ValidateRun(partial,false,error))<<error;
  EXPECT_EQ(partial.accepted_steps,198u); EXPECT_EQ(partial.attempted_steps,199u);
  EXPECT_EQ(partial.native_cell_intervals,398u);
  EXPECT_FALSE(ValidateRun(partial,true,error));
  ++partial.native_cell_intervals; EXPECT_FALSE(ValidateRun(partial,false,error));
  ::tl::qualification::qeph::wall_response::Run empty;
  empty.config=test::ConfigFor(1); empty.failure="Synthetic initialization failure";
  EXPECT_TRUE(ValidateRun(empty,false,error)); EXPECT_FALSE(ValidateRun(empty,true,error));
}
} // namespace tl::qualification::qeph::wall_response::comparison_test
