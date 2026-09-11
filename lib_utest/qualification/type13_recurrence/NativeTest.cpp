#include "Fixture.h"
#include "NativeOracle.h"
#include "lib_utest/qualification/type13/source_fixture/YarisType13SourceFixture.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cmath>
namespace type13_recurrence_test {
namespace {
void Agreement(const t::Evaluation& actual,const t::Evaluation& native) {
  const auto a=Values(actual),b=Values(native);
  for(unsigned k=0;k<a.size();++k) {
    SCOPED_TRACE(k);ASSERT_TRUE(std::isfinite(a[k]));ASSERT_TRUE(std::isfinite(b[k]));
    EXPECT_NEAR(a[k],b[k],2e-11*std::max(1e-10,std::max(std::fabs(a[k]),std::fabs(b[k]))));
  }
  for(unsigned c=0;c<6;++c)EXPECT_EQ(actual.native_history.channels[c].curve_position,native.native_history.channels[c].curve_position);
}
t::NativeHistory Virgin(const t::Reference& r) {t::NativeHistory h;h.transverse_axis=f::Column(r.axes,1);return h;}
void RunModes(Case c) {
  const double values[]={0,.025,.08,.13,.13,.125,.10,0,-.09,-.15,-.15,-.12,0,.13};
  for(unsigned mode=0;mode<6;++mode) {
    SCOPED_TRACE(mode);Case branch=c;Mode(branch,mode,0,0,.01);
    t::Evaluation actual;ASSERT_EQ(t::InitializeForce(branch.property,branch.reference,branch.nodes,actual),t::Status::Success);
    auto native=NativeEvaluate(branch.property,branch.reference,Virgin(branch.reference),branch.nodes,0,true);
    Agreement(actual,native);double previous=0;double peak=0;bool unload=false;
    for(unsigned repetition=0;repetition<4;++repetition)for(double value:values) {
      Mode(branch,mode,value,previous,.01);t::Evaluation next;
      ASSERT_EQ(t::Evaluate(branch.property,branch.reference,actual.native_history,branch.nodes,.01,next),t::Status::Success);
      const auto expected=NativeEvaluate(branch.property,branch.reference,native.native_history,branch.nodes,.01);
      Agreement(next,expected);unload|=next.signed_work_J[mode]<actual.signed_work_J[mode];
      peak=std::max(peak,next.native_history.channels[mode].accumulated_plastic_deformation);
      actual=next;native=expected;previous=value;
    }
    EXPECT_GT(peak,.001);EXPECT_TRUE(unload);
  }
}
}
TEST(Type13H1Native,AllSixIndependentHistoriesLoadHoldUnloadReloadOnDenseFrame) {RunModes(Case(true));}
TEST(Type13H1Native,OriginalFirstClosestAlignmentAndLastReferences) {
  for(const unsigned id:{2102273u,2407223u,2409378u}) {
    SCOPED_TRACE(id);Case c;
    const auto it=std::find_if(std::begin(yaris_type13_fixture::Beams),std::end(yaris_type13_fixture::Beams),
        [=](const auto& b){return b.id==id;});ASSERT_NE(it,std::end(yaris_type13_fixture::Beams));
    for(unsigned i=0;i<3;++i){const auto& xyz=yaris_type13_fixture::Nodes[it->nodes[i]].native;
      c.original.position[i]={xyz[0],xyz[1],xyz[2]};}
    t::Startup startup;ASSERT_EQ(t::InitializeElement(c.property,c.original,startup),t::Status::Success);c.reference=startup.reference;
    RunModes(c);
  }
}
TEST(Type13H1Native,FreshVersusLaterReferencePhaseAndFailureNextForceWork) {
  Case c(false,.125);t::Evaluation actual;
  ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,actual),t::Status::Success);
  auto native=NativeEvaluate(c.property,c.reference,Virgin(c.reference),c.nodes,0,true);Agreement(actual,native);
  Mode(c,0,.25,0,.01);t::Evaluation failed;
  ASSERT_EQ(t::Evaluate(c.property,c.reference,actual.native_history,c.nodes,.01,failed),t::Status::Success);
  native=NativeEvaluate(c.property,c.reference,native.native_history,c.nodes,.01);Agreement(failed,native);
  ASSERT_TRUE(failed.newly_failed);ASSERT_GT(failed.local_force_N.x,0);
  const auto wrong_phase=NativeEvaluate(c.property,c.reference,Virgin(c.reference),c.nodes,.01,true);
  EXPECT_EQ(wrong_phase.native_history.channels[0].deformation,0);
  EXPECT_NE(wrong_phase.local_force_N.x,failed.local_force_N.x);
  Mode(c,0,.30,.25,.01);t::Evaluation next;
  ASSERT_EQ(t::Evaluate(c.property,c.reference,failed.native_history,c.nodes,.01,next),t::Status::Success);
  native=NativeEvaluate(c.property,c.reference,native.native_history,c.nodes,.01);Agreement(next,native);
  EXPECT_EQ(next.local_force_N.x,0);EXPECT_GT(next.signed_work_J[0],failed.signed_work_J[0]);
}
TEST(Type13H1Native,DirectionalCurveKnotsAndExtrapolationMatchCompleteVinter) {
  Case c;const auto& curve=c.property.curve(0);const double length=2,step=.01,active=1;
  const double stiffness=c.property.channel(0).native_stiffness;
  for(const unsigned start:{0u,1u,2u,3u})for(const double query:{-2.,curve.points[1].x,0.,curve.points[3].x,2.}) {
    SCOPED_TRACE(start);
    SCOPED_TRACE(query);
    t::NativeChannelHistory old;old.curve_position=start;old.elastic_plastic_force=query*stiffness;
    auto actual=old;actual.deformation=0;
    ASSERT_EQ(t::detail::ChannelResponse(c.property,0,length,old,true,step,actual),t::Status::Success);
    const double base[5]={0,0,old.elastic_plastic_force,0,0},motion=0;double points[10],out[5];int pos=start,newpos=0;
    for(unsigned i=0;i<5;++i){points[2*i]=curve.points[i].x;points[2*i+1]=curve.points[i].y;}
    type13_h1_channel(base,&motion,&stiffness,&length,&step,&active,points,&pos,out,&newpos);
    EXPECT_EQ(actual.curve_position,static_cast<unsigned>(newpos));
    EXPECT_DOUBLE_EQ(actual.force,out[3]);EXPECT_DOUBLE_EQ(actual.accumulated_plastic_deformation,out[1]);
  }
}
TEST(Type13H1Native,CombinedSixChannelMotionAndCarriedMeanTwistUseActualMidpoint) {
  Case c(true);
  t::Evaluation actual;
  ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,actual),t::Status::Success);
  auto native=NativeEvaluate(c.property,c.reference,Virgin(c.reference),c.nodes,0,true);
  const double dt=.002;
  bool saw_plastic[6]{};
  for(unsigned step=1;step<=96;++step) {
    const double phase=.17*step;
    const auto previous=c.nodes[1].position;
    const t::Vec3 displacement{.09*std::sin(phase),.075*std::sin(phase*.7),.065*std::sin(phase*1.3)};
    c.nodes[1].position=f::Add(c.original.position[1],f::ToWorld(c.reference.axes,displacement));
    c.nodes[1].velocity=f::Divide(f::Subtract(c.nodes[1].position,previous),dt);
    const auto mean=f::Scale(f::Column(c.reference.axes,0),12.0);
    const auto relative=f::ToWorld(c.reference.axes,{8*std::cos(phase),7*std::cos(phase*.8),9*std::cos(phase*1.2)});
    c.nodes[0].angular_velocity=f::Subtract(mean,f::Scale(relative,.5));
    c.nodes[1].angular_velocity=f::Add(mean,f::Scale(relative,.5));
    const auto old=actual;
    ASSERT_EQ(t::Evaluate(c.property,c.reference,actual.native_history,c.nodes,dt,actual),t::Status::Success);
    native=NativeEvaluate(c.property,c.reference,native.native_history,c.nodes,dt);
    Agreement(actual,native);
    for(unsigned k=0;k<6;++k)saw_plastic[k]|=actual.native_history.channels[k].accumulated_plastic_deformation>1e-4;
    if(step==48) {
      auto wrong_nodes=c.nodes;
      // Deliberately replace midpoint velocities by zero; do not call this a
      // second physical trajectory. It must alter the native force packet.
      t::NativeEndpointKinematics wrong[2]={wrong_nodes[0],wrong_nodes[1]};
      wrong[0].velocity={};wrong[1].velocity={};
      const auto wrong_phase=NativeEvaluate(c.property,c.reference,old.native_history,wrong,dt);
      EXPECT_GT(std::fabs(actual.native_history.channels[1].deformation-wrong_phase.native_history.channels[1].deformation),1e-5);
    }
  }
  for(bool yielded:saw_plastic)EXPECT_TRUE(yielded);
}
TEST(Type13H1Native,RejectedIntervalRetryPreservesOwnNativeTrajectory) {
  Case c(true);t::Evaluation actual;ASSERT_EQ(t::InitializeForce(c.property,c.reference,c.nodes,actual),t::Status::Success);
  auto native=NativeEvaluate(c.property,c.reference,Virgin(c.reference),c.nodes,0,true);
  for(unsigned i=0;i<32;++i) {
    const double previous=.06*std::sin(.4*i),value=.06*std::sin(.4*(i+1));
    Mode(c,4,value,previous,.01);auto bad=actual.native_history;bad.channels[5].curve_position=4;
    const auto preserved=Values(actual);
    EXPECT_EQ(t::Evaluate(c.property,c.reference,bad,c.nodes,.01,actual),t::Status::InvalidInput);EXPECT_EQ(Values(actual),preserved);
    ASSERT_EQ(t::Evaluate(c.property,c.reference,actual.native_history,c.nodes,.01,actual),t::Status::Success);
    native=NativeEvaluate(c.property,c.reference,native.native_history,c.nodes,.01);Agreement(actual,native);
  }
}
} // namespace type13_recurrence_test
