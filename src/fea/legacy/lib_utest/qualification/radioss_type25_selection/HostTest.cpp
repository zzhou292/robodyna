// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Assertions.h"
#include "NativeOracle.h"
#include <limits>
#include <stdexcept>
namespace type25_selection_test {
TEST(Type25RetainedSelection, CompleteCurrentGeometryCorpusMatchesIndependentFullNativeStage) {
  const auto cases=Cases();ASSERT_EQ(cases.size(),139u);
  for(const auto& c:cases) {
    SCOPED_TRACE(c.name);s::NativeRetainedResult actual;
    ASSERT_EQ(s::EvaluateNativeRetained(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleRetained(Profile(),c.input,c.prior));
  }
}
TEST(Type25RetainedSelection, NativeScratchSeedsCannotBecomeValidTriangleCacheWeights) {
  auto c=FromGeometry({"triangle",type25_geometry_test::Triangle()});
  for(double seed:{17.25,-91.5}) {
    RetainedScratchObservation observed;
    const auto native=OracleRetained(Profile(),c.input,c.prior,seed,&observed);
    s::NativeRetainedResult actual;
    ASSERT_EQ(s::EvaluateNativeRetained(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,native);
    for(unsigned i=0;i<4;++i)EXPECT_TRUE(actual.sector[i].defined&s::RawBarycentricDefined);
    for(unsigned i=1;i<4;++i) {
      EXPECT_FALSE(actual.cache.sector[i].defined&s::ClampedBarycentricDefined);
      EXPECT_EQ(observed.clamped_lb[i],seed);EXPECT_EQ(observed.clamped_lc[i],seed);
      EXPECT_EQ(observed.cache_lb[i],seed);EXPECT_EQ(observed.cache_lc[i],seed);
    }
  }
}
TEST(Type25RetainedSelection, InactiveAndUnderflowProductsKeepHistoryWithOnlyDefinedChannels) {
  for(double main:{0.,-400.,std::numeric_limits<double>::denorm_min()}) {
    auto c=Basic();c.input.main_coefficient=main;
    if(main>0)c.input.secondary_coefficient=std::numeric_limits<double>::denorm_min();
    c.prior.row.irtlm[0]=0;c.prior.row.irtlm[1]=0;
    s::NativeRetainedResult actual;RetainedScratchObservation scratch;
    ASSERT_EQ(s::EvaluateNativeRetained(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    const auto native=OracleRetained(Profile(),c.input,c.prior,43.,&scratch);
    Same(actual,native);EXPECT_FALSE(actual.active);
    type25_geometry_test::Same(actual.history,c.prior);
    for(unsigned i=0;i<4;++i) {
      EXPECT_FALSE(actual.sector[i].defined&s::RawBarycentricDefined);
      EXPECT_FALSE(actual.cache.sector[i].defined&s::ClampedBarycentricDefined);
      EXPECT_EQ(scratch.raw_lb[i],43.);EXPECT_EQ(scratch.raw_lc[i],43.);
      EXPECT_EQ(scratch.cache_lb[i],43.);EXPECT_EQ(scratch.cache_lc[i],43.);
    }
  }
}
TEST(Type25RetainedSelection, SubtriangleLossAndContinuationUseActualEncodedMarkers) {
  for(unsigned old=1;old<=4;++old)for(auto point:{n::Vector{2,1,.2},n::Vector{2,3,.2},n::Vector{20,20,2}}) {
    auto c=Basic();c.prior.row.irtlm[1]=int(old);c.input.secondary=point;
    s::NativeRetainedResult actual;
    ASSERT_EQ(s::EvaluateNativeRetained(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleRetained(Profile(),c.input,c.prior));
    EXPECT_EQ(actual.history.row.irtlm[0],c.prior.row.irtlm[0]);
    EXPECT_EQ(actual.history.row.irtlm[2],c.prior.row.irtlm[2]);
    EXPECT_EQ(actual.history.row.irtlm[3],c.prior.row.irtlm[3]);
    EXPECT_EQ(actual.history.row.selection_metric[1],c.prior.row.selection_metric[1]);
  }
}
TEST(Type25RetainedSelection, InvalidSourcePriorSectorAndUnsupportedModesPreserveOutput) {
  for(unsigned fault=0;fault<8;++fault) {
    auto c=Basic();auto p=Profile();const auto prior=Sentinel();auto out=prior;
    if(fault==0)c.prior.row.irtlm[1]=0;
    if(fault==1)c.input.key.generation++;
    if(fault==2)c.input.local_main++;
    if(fault==3)c.input.secondary.x=std::numeric_limits<double>::quiet_NaN();
    if(fault==4)c.input.normal_slot[2].y=std::numeric_limits<float>::infinity();
    if(fault==5)p.foreign_rows=true;
    if(fault==6)c.input.applied_gap=1;
    if(fault==7)c.input.main_coefficient=std::numeric_limits<double>::max();
    if(fault==7)c.input.secondary_coefficient=2;
    const auto status=s::EvaluateNativeRetained(p,c.input,c.prior,&out);
    const auto expected=fault==0?s::Status::UndefinedNativeInput:
        (fault==5||fault==6?s::Status::UnsupportedProfile:
         fault==7?s::Status::NonfiniteResult:s::Status::InvalidInput);
    EXPECT_EQ(status,expected);Same(out,prior,true);
  }
}

TEST(Type25RetainedSelection, SharedNodeIdentityRejectsConflictingSignedZeroBeforePublication) {
  auto c=FromGeometry({"triangle",type25_geometry_test::Triangle()});
  c.input.main_vertices[3].z=-0.0;
  const auto prior=Sentinel();auto actual=prior;
  EXPECT_EQ(s::EvaluateNativeRetained(Profile(),c.input,c.prior,&actual),s::Status::InvalidInput);
  Same(actual,prior,true);
  EXPECT_THROW(OracleRetained(Profile(),c.input,c.prior),std::invalid_argument);
}

} // namespace type25_selection_test
