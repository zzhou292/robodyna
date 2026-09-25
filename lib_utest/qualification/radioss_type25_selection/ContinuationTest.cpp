// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ContinuationAssertions.h"
#include "NativeOracle.h"
#include <limits>
#include <cmath>
namespace type25_selection_test {
TEST(Type25Continuation, CompleteRetainedGeometryAndNativeSlidingSymmetryCorpus) {
  const auto cases=ContinuationCases();ASSERT_EQ(cases.size(),167u);
  for(const auto& c:cases) {
    SCOPED_TRACE(c.name);s::NativeContinuationResult actual;
    ASSERT_EQ(s::EvaluateNativeContinuation(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleContinuation(Profile(),c.input,c.prior));
  }
}
TEST(Type25Continuation, DistanceHysteresisAndEqualDistanceUseNativeGlobalMainTie) {
  const auto basic=BasicContinuation();
  const auto classified=OracleContinuation(Profile(),basic.input,basic.prior);
  ASSERT_TRUE(classified.active);ASSERT_GT(classified.selected_subtriangle,0);
  for(int previous_main:{2,3,4})for(bool inside:{false,true}) {
    auto c=basic;c.prior.row.irtlm[0]=previous_main;
    c.prior.row.selection_metric[0]=inside?1e20:0.;
    c.prior.row.selection_metric[1]=classified.distance_squared;
    s::NativeContinuationResult actual;
    ASSERT_EQ(s::EvaluateNativeContinuation(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleContinuation(Profile(),c.input,c.prior));
    EXPECT_EQ(actual.row_replaced,inside&&previous_main<3);
  }
  const double pivot=classified.distance_squared/1.02;
  const double lower=std::nextafter(pivot,0.),upper=std::nextafter(pivot,1.);
  bool saw_accept=false,saw_reject=false;
  for(double threshold:{std::nextafter(lower,0.),lower,pivot,upper,std::nextafter(upper,1.)}) {
    auto c=basic;c.prior.row.selection_metric[0]=threshold;
    const auto native=OracleContinuation(Profile(),c.input,c.prior);
    s::NativeContinuationResult actual;
    ASSERT_EQ(s::EvaluateNativeContinuation(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,native);saw_accept|=native.row_replaced;saw_reject|=!native.row_replaced;
  }
  EXPECT_TRUE(saw_accept);EXPECT_TRUE(saw_reject);
}
TEST(Type25Continuation, DuplicateReferenceMarksFirstSlotAndRetainsNativeConstraintCodeMeaning) {
  auto c=BasicContinuation();
  c.input.normal_reference[2]=c.input.normal_reference[0];
  c.input.sliding_reference[1]=c.input.normal_reference[2];
  c.input.sliding_reference[3]=77;
  for(int code:{0,1,2,3,4,5,6,7,8}) {
    c.input.pair.segment_type=17;c.input.secondary_skew=1;c.input.secondary_constraint=code;
    for(unsigned i=0;i<4;++i){c.input.main_constraint[i]=code;c.input.main_skew[i]=2;}
    s::NativeContinuationResult actual;
    ASSERT_EQ(s::EvaluateNativeContinuation(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleContinuation(Profile(),c.input,c.prior));
    EXPECT_EQ(actual.sliding_match[0],1);EXPECT_EQ(actual.sliding_match[2],0);
    // Native main ISKEW values are not an extra condition for this branch.
    if(code==1)EXPECT_EQ(actual.constrained_axis_mask,4u);
    if(code==8)EXPECT_EQ(actual.constrained_axis_mask,0u);
  }
}
TEST(Type25Continuation, SlidingReferenceActivatesNativeConeForBacksideOutsideGap) {
  auto c=BasicContinuation();c.input.pair.secondary={-.5,-.5,-.2};
  const auto no_slide=OracleContinuation(Profile(),c.input,c.prior);
  ASSERT_GT(no_slide.selected_subtriangle,0);
  c.input.sliding_reference[0]=c.input.normal_reference[0];
  const auto sliding=OracleContinuation(Profile(),c.input,c.prior);
  ASSERT_EQ(sliding.selected_subtriangle,0);
  s::NativeContinuationResult actual;
  ASSERT_EQ(s::EvaluateNativeContinuation(Profile(),c.input,c.prior,&actual),s::Status::Ok);
  Same(actual,sliding);EXPECT_EQ(actual.cylindrical_gap[0],0);
}
TEST(Type25Continuation, NativeInactiveAndTriangleScratchAreExplicitlyMasked) {
  for(bool inactive:{false,true})for(double seed:{17.25,-91.5}) {
    auto c=Continue(FromGeometry({"triangle",type25_geometry_test::Triangle()}));
    if(inactive)c.input.pair.main_coefficient=0;
    RetainedScratchObservation scratch;s::NativeContinuationResult actual;
    ASSERT_EQ(s::EvaluateNativeContinuation(Profile(),c.input,c.prior,&actual),s::Status::Ok);
    Same(actual,OracleContinuation(Profile(),c.input,c.prior,seed,&scratch));
    for(unsigned i=inactive?0:1;i<4;++i) {
      EXPECT_FALSE(actual.cache.sector[i].defined&s::ClampedBarycentricDefined);
      EXPECT_EQ(scratch.cache_lb[i],seed);EXPECT_EQ(scratch.cache_lc[i],seed);
    }
    if(inactive)type25_geometry_test::Same(actual.history,c.prior,true);
  }
}
TEST(Type25Continuation, InvalidAuthenticReferencesAndRangesPreserveWholeOutput) {
  for(unsigned fault=0;fault<8;++fault) {
    auto c=BasicContinuation();const auto before=ContinuationSentinel();auto actual=before;
    if(fault==0)c.input.segment_count=0;
    if(fault==1)c.input.normal_reference[0]=-1;
    if(fault==2)c.input.sliding_reference[2]=-1;
    if(fault==3)c.input.pair.boundary_ids[0]=91;
    if(fault==4)c.input.pair.segment_type=33;
    if(fault==5)c.input.pair.key.generation++;
    if(fault==6)c.prior.row.selection_metric[0]=std::numeric_limits<double>::infinity();
    if(fault==7)c.input.pair.main_coefficient=std::numeric_limits<double>::max();
    const auto status=s::EvaluateNativeContinuation(Profile(),c.input,c.prior,&actual);
    EXPECT_EQ(status,fault==7?s::Status::NonfiniteResult:s::Status::InvalidInput);
    Same(actual,before,true);
  }
}
} // namespace type25_selection_test
