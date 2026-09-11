#include "Fixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>
namespace type13_recurrence_test {
TEST(Type13H1,StartupUsesExplicitOriginalZeroDurationPacket) {
  Case c(true);t::Evaluation e;
  for(auto& node:c.nodes)node.velocity={8,2,-3};
  ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,e),t::Status::Success);
  for(const auto& h:e.native_history.channels){EXPECT_EQ(h.force,0);EXPECT_EQ(h.deformation,0);EXPECT_EQ(h.signed_work,0);}
  EXPECT_TRUE(e.native_history.active);EXPECT_GT(e.stability.critical_dt_s,0);
  auto before=Values(e);c.nodes[1].velocity.x+=1;
  EXPECT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,e),t::Status::UnsupportedScope);EXPECT_EQ(Values(e),before);
  c.nodes[1].velocity.x-=1;c.nodes[0].angular_velocity.z=1;
  EXPECT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,e),t::Status::UnsupportedScope);EXPECT_EQ(Values(e),before);
}
TEST(Type13H1,AllSixModesYieldUnloadReverseAndRetainIndependentHistory) {
  const double path[]={.025,.08,.10,.10,.095,.075,0,-.06,-.12,-.12,0,.08};
  for(unsigned channel=0;channel<6;++channel) {
    SCOPED_TRACE(channel);Case c;t::Evaluation old;
    ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,old),t::Status::Success);
    double previous=0;bool unloaded=false;double peak_plastic=0;
    for(double value:path) {
      Mode(c,channel,value,previous,.01);t::Evaluation next;
      ASSERT_EQ(t::Evaluate(c.property,c.reference,old.native_history,c.nodes,.01,next),t::Status::Success);
      unloaded|=next.signed_work_J[channel]<old.signed_work_J[channel];
      peak_plastic=std::fmax(peak_plastic,next.native_history.channels[channel].accumulated_plastic_deformation);
      old=next;previous=value;
    }
    EXPECT_GT(peak_plastic,.001);EXPECT_TRUE(unloaded);EXPECT_TRUE(old.native_history.active);
    t::NativeHistory reset;reset.transverse_axis=old.native_history.transverse_axis;t::Evaluation wrong;
    ASSERT_EQ(t::Evaluate(c.property,c.reference,reset,c.nodes,.01,wrong),t::Status::Success);
    EXPECT_NE(old.native_history.channels[channel].elastic_plastic_force,wrong.native_history.channels[channel].elastic_plastic_force);
  }
}
TEST(Type13H1,FailureAtOneRetainsCurrentForceThenNextMaskedWork) {
  Case c(false,.125);t::Evaluation initial,failed,after;
  ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,initial),t::Status::Success);
  Mode(c,0,.25,0,.01);
  ASSERT_EQ(t::Evaluate(c.property,c.reference,initial.native_history,c.nodes,.01,failed),t::Status::Success);
  EXPECT_TRUE(failed.newly_failed);EXPECT_FALSE(failed.native_history.active);
  EXPECT_EQ(failed.native_history.failure_criterion,1);EXPECT_GT(failed.local_force_N.x,0);
  Mode(c,0,.30,.25,.01);
  ASSERT_EQ(t::Evaluate(c.property,c.reference,failed.native_history,c.nodes,.01,after),t::Status::Success);
  EXPECT_FALSE(after.newly_failed);EXPECT_EQ(after.local_force_N.x,0);
  EXPECT_GT(after.native_history.channels[0].elastic_plastic_force,0);
  const double expected=failed.native_history.channels[0].signed_work+
      (after.native_history.channels[0].deformation-failed.native_history.channels[0].deformation)*failed.native_history.channels[0].force*.5;
  EXPECT_NEAR(after.native_history.channels[0].signed_work,expected,2e-12*std::fabs(expected));
  EXPECT_GT(after.signed_work_J[0],failed.signed_work_J[0]);
}
TEST(Type13H1,LateSixthChannelFailurePreservesOutputAndExactRetryIncludingAlias) {
  Case c;t::Evaluation accepted;
  ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,accepted),t::Status::Success);
  Mode(c,3,.04,0,.01);
  t::Evaluation expected;ASSERT_EQ(t::Evaluate(c.property,c.reference,accepted.native_history,c.nodes,.01,expected),t::Status::Success);
  auto bad=accepted.native_history;bad.channels[5].accumulated_plastic_deformation=std::numeric_limits<double>::max();
  t::Evaluation output=accepted;const auto before=Values(output);
  EXPECT_EQ(t::Evaluate(c.property,c.reference,bad,c.nodes,.01,output),t::Status::NonfiniteResult);EXPECT_EQ(Values(output),before);
  ASSERT_EQ(t::Evaluate(c.property,c.reference,output.native_history,c.nodes,.01,output),t::Status::Success);
  EXPECT_EQ(Values(output),Values(expected));
  bad=accepted.native_history;bad.channels[5].curve_position=4;
  EXPECT_EQ(t::Evaluate(c.property,c.reference,bad,c.nodes,.01,output),t::Status::InvalidInput);EXPECT_EQ(Values(output),Values(expected));
}
TEST(Type13H1,NativeExplicitRateOverflowAndCollapsedFrameFailClosed) {
  Case c;t::Evaluation e;ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,e),t::Status::Success);
  const auto before=Values(e);c.nodes[1].position.x=2.1;
  EXPECT_EQ(t::Evaluate(c.property,c.reference,e.native_history,c.nodes,1e-320,e),t::Status::NonfiniteResult);
  EXPECT_EQ(Values(e),before);c.nodes[1].position=c.nodes[0].position;
  EXPECT_EQ(t::Evaluate(c.property,c.reference,e.native_history,c.nodes,.01,e),t::Status::DegenerateGeometry);
  EXPECT_EQ(Values(e),before);
}
TEST(Type13H1,CurrentFrameWrenchesBalanceForceAndFiniteLengthMoment) {
  Case c(true);t::Evaluation e;ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,e),t::Status::Success);
  Mode(c,1,.08,0,.01);ASSERT_EQ(t::Evaluate(c.property,c.reference,e.native_history,c.nodes,.01,e),t::Status::Success);
  const auto force=f::Add(e.endpoints[0].force_N,e.endpoints[1].force_N);
  const auto chord=f::Scale(f::Subtract(c.nodes[1].position,c.nodes[0].position),c.property.units().length_to_m);
  const auto moment=f::Add(f::Add(e.endpoints[0].couple_Nm,e.endpoints[1].couple_Nm),f::Cross(chord,e.endpoints[1].force_N));
  EXPECT_EQ(f::Norm(force),0);EXPECT_LT(f::Norm(moment),1e-11);
}
} // namespace type13_recurrence_test
