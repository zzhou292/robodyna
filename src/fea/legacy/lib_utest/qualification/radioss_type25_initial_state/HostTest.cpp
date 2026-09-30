// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>
namespace initial_state_test {
void Compare(Fixture& fixture,int sharp,bool expect_warm=true) {
  const auto expected=InitialHistory(fixture.Input(),fixture.topology,fixture.gaps,fixture.candidates,sharp);
  std::vector<init::Winner> winners(fixture.secondary.size());
  for(std::size_t i=0;i<fixture.candidates.size();++i) {
    SCOPED_TRACE(i);
    const auto input=fixture.Pair(i,sharp);init::PairResult value;
    ASSERT_EQ(init::EvaluatePair(input,&value),init::Status::Ok);
    ASSERT_EQ(init::Consider(value,winners[input.geometry.key.history_index]),init::Status::Ok);
  }
  std::size_t warm=0;
  for(std::size_t row=0;row<winners.size();++row) {
    SCOPED_TRACE(row);
    Same(winners[row],expected.before_pwr[row]);
    EXPECT_EQ(Bits(winners[row].distance_squared),Bits(expected.nearest_distance[row]));
    init::FinalizeInacti5(winners[row]);Same(winners[row],expected.rows[row]);
    EXPECT_EQ(expected.initial_contact[row],0);warm+=winners[row].row.irtlm[0]>0;
  }
  if(expect_warm)EXPECT_GT(warm,0u);else EXPECT_EQ(warm,0u);
}
TEST(InitialStatePair, WholeStarterWarmColdQ4T3AndWarpedSourcePhasesMatch) {
  for(unsigned mode=0;mode<3;++mode)for(bool warped:{false,true})for(int sharp:{1,2}) {
    SCOPED_TRACE(mode);
    SCOPED_TRACE(warped);
    SCOPED_TRACE(sharp);
    Fixture fixture(mode,warped);Compare(fixture,sharp);
  }
}
TEST(InitialStatePair, SourceOrderReversalPreservesNativeDistanceAndGlobalTie) {
  for(unsigned mode:{0u,1u,2u}) {
    Fixture fixture(mode);const auto before=InitialHistory(fixture.Input(),fixture.topology,fixture.gaps,fixture.candidates);
    std::reverse(fixture.candidates.begin(),fixture.candidates.end());
    const auto after=InitialHistory(fixture.Input(),fixture.topology,fixture.gaps,fixture.candidates);
    ASSERT_EQ(before.rows.size(),after.rows.size());
    for(std::size_t row=0;row<before.rows.size();++row) {
      for(unsigned k=0;k<4;++k)EXPECT_EQ(before.rows[row].irtlm[k],after.rows[row].irtlm[k]);
      EXPECT_EQ(Bits(before.rows[row].penetration[4]),Bits(after.rows[row].penetration[4]));
    }
    Compare(fixture,1);
  }
}
TEST(InitialStatePair, NumericalSolidAndCoatingRolesFollowWholeNativeEligibility) {
  // Explicit value packets, not a source-generated mixed/support certificate.
  for(int code:{0,1,2,3}) {
    Fixture fixture;
    const int encoded=int(fixture.topology.mains.size()+1);
    const int role=code==0?0:code==1?1:code==2?encoded:-encoded;
    for(auto& main:fixture.topology.mains)main.segment_type=role;
    Compare(fixture,1,code!=3);
  }
}
TEST(InitialStatePair, WarmMainCoefficientSignIsNotAStarterPairFilter) {
  Fixture fixture(0,true);
  // Initial PEN3/PWR5 consumes no main-STF argument. The source inventory's
  // actual support/part rules are a separate required stage, not an ABS rule.
  for(double coefficient:{-210000.,0.,210000.}) {
    SCOPED_TRACE(coefficient);
    std::fill(fixture.mesh.coefficients.begin(),fixture.mesh.coefficients.end(),coefficient);
    Compare(fixture,1);
  }
}
TEST(InitialStatePair, SignedZeroGapsAndExactPlaneKeepWholeNativeOffsetBits) {
  for(double zero:{0.,-0.}) {
    SCOPED_TRACE(std::signbit(zero));
    Fixture fixture;
    for(auto& row:fixture.secondary)row.gap=zero;
    for(auto& main:fixture.gaps)main.fill(zero);
    for(const auto& row:fixture.secondary)fixture.mesh.positions[3*row.node+2]=0.;
    Compare(fixture,1,false);
  }
}
TEST(InitialStatePair, NativeUnreadCoefficientsAndUnboundBisectorsDoNotBecomeAdmissionOperands) {
  Fixture fixture;auto input=fixture.Pair(0);init::PairResult baseline,actual;
  for(unsigned k=0;k<4;++k) {
    input.geometry.boundary_ids[k]=0;
    for(unsigned j=0;j<2;++j)input.geometry.vertex_bisector[k][j]={};
  }
  ASSERT_EQ(init::EvaluatePair(input,&baseline),init::Status::Ok);
  const double poison=std::numeric_limits<double>::quiet_NaN();
  input.geometry.main_coefficient=poison;input.geometry.secondary_coefficient=poison;
  for(unsigned k=0;k<4;++k)for(unsigned j=0;j<2;++j)
    input.geometry.vertex_bisector[k][j]={float(poison),float(poison),float(poison)};
  ASSERT_EQ(init::EvaluatePair(input,&actual),init::Status::Ok);
  EXPECT_EQ(actual.considered,baseline.considered);EXPECT_EQ(actual.sector,baseline.sector);
  EXPECT_EQ(Bits(actual.distance_squared),Bits(baseline.distance_squared));
  EXPECT_EQ(Bits(actual.penetration_offset),Bits(baseline.penetration_offset));
  input.geometry.boundary_ids[0]=1;actual.local_main=777;
  EXPECT_EQ(init::EvaluatePair(input,&actual),init::Status::InvalidInput);
  EXPECT_EQ(actual.local_main,777);
}
TEST(InitialStatePair, InvalidProfileAndLateGeometryPreserveOutputThenRetry) {
  Fixture fixture;const auto input=fixture.Pair(0);init::PairResult output;output.local_main=777;
  auto wrong=input;wrong.profile.initial_penetration=0;
  EXPECT_EQ(init::EvaluatePair(wrong,&output),init::Status::UnsupportedProfile);
  EXPECT_EQ(output.local_main,777);
  wrong=input;wrong.geometry.main_vertices[3].x=std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(init::EvaluatePair(wrong,&output),init::Status::InvalidInput);
  EXPECT_EQ(output.local_main,777);
  EXPECT_EQ(init::EvaluatePair(input,&output),init::Status::Ok);
}
TEST(InitialStateWinner, NativeDistanceTieAndPwrZeroResetAreDistinctFromMaximumPenetration) {
  init::Winner value;init::PairResult a;a.considered=true;a.key.main_segment=3;a.local_main=2;a.sector=1;
  a.distance_squared=.25;a.penetration_offset=.1;
  ASSERT_EQ(init::Consider(a,value),init::Status::Ok);
  a.key.main_segment=4;a.distance_squared=1;a.penetration_offset=9;
  ASSERT_EQ(init::Consider(a,value),init::Status::Ok);
  EXPECT_EQ(value.row.irtlm[0],3);
  a.distance_squared=.25;a.penetration_offset=0;
  ASSERT_EQ(init::Consider(a,value),init::Status::Ok);
  EXPECT_EQ(value.row.irtlm[0],4);
  init::FinalizeInacti5(value);EXPECT_EQ(value.row.irtlm[0],0);
  EXPECT_EQ(init::MapPreparedMain(0,0,value),init::Status::Ok);
  a.penetration_offset=.1;a.distance_squared=.01;
  ASSERT_EQ(init::Consider(a,value),init::Status::Ok);
  const auto before=value.row;
  EXPECT_EQ(init::MapPreparedMain(2,0,value),init::Status::InvalidInput);
  EXPECT_EQ(value.row.irtlm[3],before.irtlm[3]);
  EXPECT_EQ(init::MapPreparedMain(2,1,value),init::Status::Ok);
  EXPECT_EQ(value.row.irtlm[2],2);EXPECT_EQ(value.row.irtlm[3],1);
}
}
