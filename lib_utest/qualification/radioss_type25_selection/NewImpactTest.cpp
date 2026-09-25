// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NewImpactAssertions.h"
#include "NativeOracle.h"
#include <climits>
#include <limits>
#include <cmath>
namespace type25_selection_test {
TEST(Type25NewImpact, CompleteGeometryRolesMotionAndCrossingCorpusMatchesNative) {
  const auto cases=NewImpactCases();ASSERT_EQ(cases.size(),273u);
  bool primary=false,opposite=false,recontact=false;
  for(const auto& c:cases) {
    SCOPED_TRACE(c.name);s::NativeNewImpactResult actual;
    ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleNewImpact(Profile(),c.input,c.prior));
    primary|=actual.primary.intersection!=0;opposite|=actual.opposite.intersection!=0;
    recontact|=actual.recontact_intersection!=0;
  }
  EXPECT_TRUE(primary);EXPECT_TRUE(opposite);EXPECT_TRUE(recontact);
}
TEST(Type25NewImpact, MovingMainCrossingUsesActualDt1AndAllCurrentVelocities) {
  auto c=BasicNewImpact();const auto stationary=OracleNewImpact(Profile(),c.input,c.prior);
  ASSERT_EQ(stationary.opposite.intersection,0);
  for(auto& velocity:c.input.main_velocity)velocity={0,0,-1000};
  const auto moving=OracleNewImpact(Profile(),c.input,c.prior);
  ASSERT_EQ(moving.opposite.intersection,1);
  s::NativeNewImpactResult actual;
  ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&actual),s::Status::Ok);
  Same(actual,moving);
  c.input.previous_dt=0;
  const auto zero_dt=OracleNewImpact(Profile(),c.input,c.prior);
  ASSERT_EQ(zero_dt.opposite.intersection,0);
  ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&actual),s::Status::Ok);
  Same(actual,zero_dt);
}
TEST(Type25NewImpact, InactiveProductsStillPublishAllNativeDefinedProjectionAndClearedCache) {
  for(bool triangle:{false,true})for(double coefficient:{0.,-400.})for(double seed:{17.25,-91.5}) {
    auto c=Impact(FromGeometry({"inactive",triangle?type25_geometry_test::Triangle():type25_geometry_test::Quad()}));
    c.input.pair.main_coefficient=coefficient;
    NewImpactScratchObservation scratch;s::NativeNewImpactResult actual;
    ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleNewImpact(Profile(),c.input,c.prior,seed,&scratch));
    EXPECT_FALSE(actual.active);EXPECT_EQ(actual.scalar_defined,0u);
    EXPECT_EQ(scratch.penetration,seed);EXPECT_EQ(scratch.lb,seed);EXPECT_EQ(scratch.lc,seed);
    EXPECT_EQ(scratch.far,-91);
    type25_geometry_test::Same(actual.history,c.prior,true);
    for(unsigned i=0;i<4;++i) {
      EXPECT_EQ(actual.projection[i].defined,s::DistanceSquaredDefined|s::RawBarycentricDefined|s::ClampedBarycentricDefined);
      EXPECT_EQ(actual.cache.sector[i].defined,s::FarDefined|s::PenetrationDefined|s::ClampedBarycentricDefined);
      EXPECT_EQ(actual.cache.sector[i].penetration,0.);EXPECT_EQ(actual.cache.sector[i].lb,0.);
      EXPECT_EQ(actual.cache.sector[i].lc,0.);EXPECT_EQ(actual.cache.sector[i].far,0);
    }
  }
}
TEST(Type25NewImpact, EqualSidesRejectWhileOppositeCacheRewriteDoesNotRequireRowWin) {
  for(bool triangle:{false,true}) {
    auto c=Impact(FromGeometry({"equal-side",triangle?type25_geometry_test::Triangle():type25_geometry_test::Quad()}));
    c.input.pair.secondary.z=0;
    const auto native=OracleNewImpact(Profile(),c.input,c.prior);
    ASSERT_GT(native.primary.subtriangle,0);ASSERT_GT(native.opposite.subtriangle,0);
    ASSERT_GT(native.primary.penetration[native.primary.subtriangle-1],0.);
    ASSERT_EQ(native.primary.penetration[native.primary.subtriangle-1],
        native.opposite.penetration[native.opposite.subtriangle-1]);
    s::NativeNewImpactResult actual;
    ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,native);EXPECT_EQ(actual.selected_side,s::ImpactSide::None);
    EXPECT_EQ(actual.scalar_defined,s::ImpactPenetrationDefined);
    c.input.pair.secondary.z=-.2;c.prior.row.selection_metric[0]=1e20;
    const auto opposite=OracleNewImpact(Profile(),c.input,c.prior);
    ASSERT_EQ(opposite.selected_side,s::ImpactSide::Opposite);ASSERT_FALSE(opposite.row_replaced);
    ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,opposite);EXPECT_EQ(actual.source_key.main_segment,c.input.pair.key.main_segment);
    EXPECT_EQ(actual.cache.key.main_segment,c.input.opposite.global_main);
    EXPECT_EQ(actual.cache.local_main,c.input.opposite.local_main);
    type25_geometry_test::Same(actual.history,c.prior,true);
  }
}
TEST(Type25NewImpact, StrictPenetrationAndGlobalMainTieAreNativeRowDecisions) {
  const auto base=BasicNewImpact();const auto result=OracleNewImpact(Profile(),base.input,base.prior);
  ASSERT_GT(result.penetration,0.);ASSERT_EQ(result.selected_side,s::ImpactSide::Primary);
  for(int main:{2,3,4})for(double metric:{std::nextafter(result.penetration,0.),
      result.penetration,std::nextafter(result.penetration,1.)}) {
    auto c=base;c.prior.row.irtlm[0]=-main;c.prior.row.selection_metric[0]=metric;
    s::NativeNewImpactResult actual;
    ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleNewImpact(Profile(),c.input,c.prior));
    EXPECT_EQ(actual.row_replaced,metric<result.penetration||
        (metric==result.penetration&&main<c.input.pair.key.main_segment));
  }
}
TEST(Type25NewImpact, OriginalRecontactControlAndBoundedReferenceStorageRemainVisible) {
  auto c=BasicNewImpact();SetImpactRole(c,0);c.input.pair.secondary.z=-1e-6;
  c.input.pair.secondary_gap=0;for(auto& gap:c.input.pair.main_gap)gap=0;
  c.input.pair.main_gap_max=0;
  for(int initial:{-1,0,1}) {
    c.input.pair.initial_contact_flag=initial;
    NewImpactOracleStorage storage;s::NativeNewImpactResult actual;
    const auto native=OracleNewImpact(Profile(),c.input,c.prior,0,nullptr,&storage);
    ASSERT_EQ(s::EvaluateNativeNewImpact(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,native);EXPECT_EQ(native.recontact_intersection,initial<0?0:1);
    EXPECT_EQ(storage.main_slots,std::size_t(c.input.segment_count));
    EXPECT_EQ(storage.table_bytes,264*storage.main_slots+28*storage.reference_slots);
    EXPECT_LT(storage.table_bytes,1u<<20);
  }
}
TEST(Type25NewImpact, InvalidPacketAndNonfiniteResultPreserveEntirePriorOutput) {
  for(unsigned fault=0;fault<9;++fault) {
    auto c=BasicNewImpact();auto profile=Profile();const auto before=NewImpactSentinel();auto actual=before;
    if(fault==0)c.input.pair.key.generation++;
    if(fault==1)c.input.previous_dt=-1;
    if(fault==2)c.input.opposite.local_main=c.input.pair.local_main;
    if(fault==3)c.input.opposite.main_node_ids[0]++;
    if(fault==4)c.input.pair.segment_type=-33;
    if(fault==5)c.prior.row.irtlm[0]=INT_MIN;
    if(fault==6)c.input.secondary_velocity.z=std::numeric_limits<double>::infinity();
    if(fault==7)profile.foreign_rows=true;
    if(fault==8)c.input.pair.main_coefficient=std::numeric_limits<double>::max();
    EXPECT_EQ(s::EvaluateNativeNewImpact(profile,c.input,c.prior,&actual),
        fault==7?s::Status::UnsupportedProfile:fault==8?s::Status::NonfiniteResult:s::Status::InvalidInput);
    Same(actual,before,true);
  }
}
} // namespace type25_selection_test
