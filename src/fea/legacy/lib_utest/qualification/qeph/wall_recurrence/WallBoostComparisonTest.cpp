#include "WallBoostTestFixture.h"
#include <limits>

namespace tl::qualification::qeph::wall_recurrence {
namespace bt=boost_test;
TEST(QephWallBoost, KnownVelocityLiftAndBothBranchOperatorsAreComparedForBothSigns) {
  for(unsigned cells:{1u,2u}) {
    const auto zero=bt::Raw(cells,0); const auto summary=bt::Summary(zero);
    WallZeroBoostReference reference; std::string error;
    ASSERT_TRUE(PrepareWallZeroBoostReference(zero,summary,reference,error))<<error;
    for(double velocity:{-8.,8.}) {
      const auto raw=bt::Raw(cells,velocity); const auto result=CompareWallBoost(reference,raw,bt::Summary(raw));
      ASSERT_TRUE(result.input_valid)<<result.diagnostic;
      for(unsigned s=0;s<6;++s) for(const auto& a:result.steps[s].amplitudes) {
        EXPECT_TRUE(a.complete); EXPECT_TRUE(a.passed); EXPECT_EQ(a.lift_residual_max,0);
        EXPECT_TRUE(a.native_matrix.passed);
        for(const auto& matrix:a.branch_matrices) EXPECT_TRUE(matrix.passed);
        for(unsigned node=0;node<raw.model.native().nodes;++node) {
          const auto x=raw.model.coordinate(recurrence::Group::Position,node,0);
          const auto v=raw.model.coordinate(recurrence::Group::Velocity,node,0);
          EXPECT_EQ(a.baselines[1].expected[x],recurrence::Steps[s]*velocity/raw.model.native().dictionary[x].scale);
          EXPECT_EQ(a.baselines[1].expected[v],velocity/raw.model.native().dictionary[v].scale);
        }
      }
    }
  }
}
TEST(QephWallBoost, LateTupleContextFailurePreservesZeroReferenceAndAllowsRetry) {
  const auto zero=bt::Raw(1,0); const auto clean=bt::Summary(zero); std::string error;
  WallZeroBoostReference reference;
  auto partial_raw=zero; auto partial=clean;
  partial.steps[5].complete=false; partial.steps[5].passed=false;
  partial.steps[5].amplitudes[2].complete=false; partial.steps[5].amplitudes[2].passed=false;
  partial_raw.steps[5].native[2].derivative.complete=false;
  partial_raw.steps[5].native[2].derivative.full.resize(110,109);
  EXPECT_FALSE(PrepareWallZeroBoostReference(partial_raw,partial,reference,error));
  EXPECT_FALSE(reference.prepared());
  auto bad=clean; bad.steps[5].diagonal[0]*=2;
  EXPECT_FALSE(PrepareWallZeroBoostReference(zero,bad,reference,error)); EXPECT_FALSE(reference.prepared());
  ASSERT_TRUE(PrepareWallZeroBoostReference(zero,clean,reference,error))<<error;
  EXPECT_FALSE(PrepareWallZeroBoostReference(zero,clean,reference,error)); EXPECT_TRUE(reference.prepared());
  const auto boost=bt::Raw(1,8); const auto summary=bt::Summary(boost);
  for(unsigned fault=0;fault<7;++fault) {
    SCOPED_TRACE(fault);
    auto raw=boost; auto changed=summary;
    if(fault==0) changed.cells=2;
    if(fault==1) changed.dimension=194;
    if(fault==2) changed.steps[5].h=recurrence::Steps[4];
    if(fault==3) changed.steps[5].diagonal[0]*=2;
    if(fault==4) ++changed.steps[5].windows[8].entry_base_epoch;
    if(fault==5) raw.steps[5].native[2].velocity.x=-8;
    if(fault==6) changed.steps[5].amplitudes[2].gains[10].weighted=std::numeric_limits<double>::infinity();
    EXPECT_FALSE(CompareWallBoost(reference,raw,changed).input_valid);
  }
  const auto retry=CompareWallBoost(reference,boost,summary);
  ASSERT_TRUE(retry.input_valid); EXPECT_TRUE(retry.steps[5].passed);
}
TEST(QephWallBoost, WrongBaselineUnitsMatrixAndWeightedGainRemainVisibleWithoutRawGainGate) {
  const auto zero=bt::Raw(1,0); WallZeroBoostReference reference; std::string error;
  ASSERT_TRUE(PrepareWallZeroBoostReference(zero,bt::Summary(zero),reference,error))<<error;
  const auto clean=bt::Raw(1,8); const auto summary=bt::Summary(clean);
  for(unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault);
    auto raw=clean; auto changed=summary;
    if(fault==0) {
      const auto v=raw.model.coordinate(recurrence::Group::Velocity,0,0);
      raw.steps[5].native[2].baseline[v]=8; // Physical m/s in a normalized slot.
    }
    if(fault==1) raw.steps[5].native[2].derivative.full(0,0)+=.01;
    if(fault==2) changed.steps[5].amplitudes[2].gains[10].weighted=2;
    if(fault==3) changed.steps[5].amplitudes[2].gains[10].raw=1000;
    const auto result=CompareWallBoost(reference,raw,changed);
    ASSERT_TRUE(result.input_valid)<<result.diagnostic;
    EXPECT_TRUE(result.steps[0].passed); EXPECT_TRUE(result.steps[5].complete);
    const auto& last=result.steps[5].amplitudes[2];
    if(fault==0) { EXPECT_FALSE(last.baselines[1].passed); EXPECT_GT(last.lift_residual_max,1); }
    if(fault==1) { EXPECT_FALSE(last.native_matrix.passed); EXPECT_FALSE(last.branch_matrices[0].passed); }
    if(fault==2) EXPECT_FALSE(last.weighted_gains[10].passed);
    if(fault==3) { EXPECT_FALSE(last.raw_gains[10].passed); EXPECT_TRUE(last.weighted_gains[10].passed); }
    EXPECT_EQ(last.passed,fault==3); EXPECT_EQ(result.steps[5].passed,fault==3);
  }
}
TEST(QephWallBoost, CompactPartialAnalysisPreservesStagedOutputAndIgnoresGlobalVerdict) {
  const auto raw=job_test::Job(); const auto analysis=AnalyzeWallRawJob(raw);
  WallJobSummary output; output.cells=99; std::string error;
  auto invalid=analysis; invalid.steps[5].h=0;
  EXPECT_FALSE(CompactWallJobAnalysis(raw,invalid,output,error)); EXPECT_EQ(output.cells,99);
  ASSERT_TRUE(CompactWallJobAnalysis(raw,analysis,output,error))<<error;
  EXPECT_EQ(output.cells,1); EXPECT_FALSE(output.steps[0].complete); EXPECT_FALSE(output.steps[5].passed);
  auto changed=analysis; changed.complete=true; changed.passed=true;
  WallJobSummary repeated;
  ASSERT_TRUE(CompactWallJobAnalysis(raw,changed,repeated,error))<<error;
  for(unsigned s=0;s<6;++s) {
    EXPECT_EQ(repeated.steps[s].complete,output.steps[s].complete);
    EXPECT_EQ(repeated.steps[s].passed,output.steps[s].passed);
  }
}
} // namespace tl::qualification::qeph::wall_recurrence
