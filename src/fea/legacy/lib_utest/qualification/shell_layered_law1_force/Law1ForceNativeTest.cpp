#include "Law1ForceTestSupport.h"
namespace layered_law1_force_test {
TEST(LayeredLaw1ForceNative,QephLoadedRotatingHeldReversedNativeHistoriesForcesAndWork) {
  for(unsigned scenario=0;scenario<3;++scenario) {
    auto input=qeph_startup_test::Case(5);const auto material=Material(input);
    q::ReferenceData r;nq::Reference nr;
    ASSERT_EQ(q::InitializeReference(input,r),q::Status::kSuccess);
    ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(input),nr),nq::Status::kSuccess);
    q::LayeredLaw1History accepted;native::QephHistory expected;
    ASSERT_EQ(q::InitializeLayeredLaw1History(r,material,{},accepted),q::Status::kSuccess);
    ASSERT_EQ(nq::InitializeHistory(nr,{},expected.shell),nq::Status::kSuccess);
    path::Path motion{scenario!=0,scenario==2?2u:1u};motion.angular_speed=18000;
    double max_thickness=0,preload=0;
    for(;motion.step<160*motion.refinement;++motion.step) {
      SCOPED_TRACE(scenario);
      SCOPED_TRACE(motion.step);
      const auto in=motion.Interval(input);q::LayeredLaw1ForceTrial a;native::QephTrial n;
      ASSERT_EQ(q::EvaluateLayeredLaw1Force(r,material,accepted,in,a),q::Status::kSuccess);
      ASSERT_EQ(native::Evaluate(nr,expected,qeph_kinematics_test::NativeInterval(in),n),nq::Status::kSuccess);
      qeph_force_port_test::ForceAgreement(a.force,n.shell,input,in,path::cv::Tolerance);Points(a.proposed_section,n.points);
      ASSERT_FALSE(::testing::Test::HasFailure());
      if(scenario==1&&motion.step==80) {
        native::QephTrial wrong;
        ASSERT_EQ(native::Evaluate(nr,expected,qeph_kinematics_test::NativeInterval(motion.Interval(input,true)),wrong),nq::Status::kSuccess);
        EXPECT_GT(ForceDifference(n.shell,wrong.shell),1.e-5); // Endpoint rates are a detectable wrong-phase control.
        auto reset=expected;reset.points={};
        ASSERT_EQ(nq::InitializeHistory(nr,expected.shell.stamp(),reset.shell),nq::Status::kSuccess);
        ASSERT_EQ(native::Evaluate(nr,reset,qeph_kinematics_test::NativeInterval(in),wrong),nq::Status::kSuccess);
        EXPECT_GT(std::abs(wrong.points[0][0]-n.points[0][0]),1.e6); // Lost native prestress is observable.
        auto corrupt=expected;corrupt.points[2][4]=std::numeric_limits<double>::quiet_NaN();
        const auto before=Bytes(wrong);const auto history_before=Bytes(corrupt);
        EXPECT_NE(native::Evaluate(nr,corrupt,qeph_kinematics_test::NativeInterval(in),wrong),nq::Status::kSuccess);
        EXPECT_EQ(Bytes(wrong),before);EXPECT_EQ(Bytes(corrupt),history_before);
        ASSERT_EQ(native::Evaluate(nr,expected,qeph_kinematics_test::NativeInterval(in),wrong),nq::Status::kSuccess);
        EXPECT_EQ(wrong.points,n.points);EXPECT_EQ(ForceDifference(wrong.shell,n.shell),0.);
      }
      max_thickness=std::max(max_thickness,std::abs(n.shell.proposed_history.data().thickness-input.thickness));
      if(motion.step+1==64*motion.refinement)preload=n.points[0][0];
      accepted={a.force.proposed_history,a.proposed_section};expected={n.shell.proposed_history,n.points};
    }
    EXPECT_GT(max_thickness,1.e-6);EXPECT_GT(std::abs(expected.points[0][0]-preload),1.e7);
    EXPECT_GT(std::abs(expected.shell.data().hourglass_viscous_work),1.e-8);
    EXPECT_GT(std::abs(expected.shell.data().internal_work[0]),1.e-6);
    if(scenario)EXPECT_GT(motion.Angle(motion.time()),1.6);
  }
}
TEST(LayeredLaw1ForceNative,T3LoadedRotatingHeldReversedNativeHistoriesForcesAndWork) {
  for(unsigned scenario=0;scenario<3;++scenario) {
    auto input=t3_port_test::Triangle(.02,1);const auto material=Material(input,10e9);
    t::ReferenceData r;nt::Reference nr;
    ASSERT_EQ(t::InitializeReference(input,r),t::Status::kSuccess);
    ASSERT_EQ(nt::Initialize(t3_port_test::Native(input),nr),nt::Status::kSuccess);
    t::LayeredLaw1History accepted;native::T3History expected;
    ASSERT_EQ(t::InitializeLayeredLaw1History(r,material,{},accepted),t::Status::kSuccess);
    ASSERT_EQ(nt::InitializeHistory(nr,{},expected.shell),nt::Status::kSuccess);
    path::Path motion{scenario!=0,scenario==2?2u:1u};motion.angular_speed=18000;
    double max_thickness=0,preload=0;
    for(;motion.step<160*motion.refinement;++motion.step) {
      SCOPED_TRACE(scenario);
      SCOPED_TRACE(motion.step);
      const auto in=motion.Interval(input);t::LayeredLaw1ForceTrial a;native::T3Trial n;
      ASSERT_EQ(t::EvaluateLayeredLaw1Force(r,material,accepted,in,a),t::Status::kSuccess);
      ASSERT_EQ(native::Evaluate(nr,expected,t3_port_test::Native(in),n),nt::Status::kSuccess);
      t3_force_port_test::Agreement(r,in,a.force,n.shell,path::cv::Tolerance);Points(a.proposed_section,n.points);
      ASSERT_FALSE(::testing::Test::HasFailure());
      if(scenario==1&&motion.step==80) {
        native::T3Trial wrong;
        ASSERT_EQ(native::Evaluate(nr,expected,t3_port_test::Native(motion.Interval(input,true)),wrong),nt::Status::kSuccess);
        EXPECT_GT(ForceDifference(n.shell,wrong.shell),1.e-5); // Endpoint rates are a detectable wrong-phase control.
        auto reset=expected;reset.points={};
        ASSERT_EQ(nt::InitializeHistory(nr,expected.shell.stamp(),reset.shell),nt::Status::kSuccess);
        ASSERT_EQ(native::Evaluate(nr,reset,t3_port_test::Native(in),wrong),nt::Status::kSuccess);
        EXPECT_GT(std::abs(wrong.points[0][0]-n.points[0][0]),1.e6); // Lost native prestress is observable.
        auto corrupt=expected;corrupt.points[2][4]=std::numeric_limits<double>::quiet_NaN();
        const auto before=Bytes(wrong);const auto history_before=Bytes(corrupt);
        EXPECT_NE(native::Evaluate(nr,corrupt,t3_port_test::Native(in),wrong),nt::Status::kSuccess);
        EXPECT_EQ(Bytes(wrong),before);EXPECT_EQ(Bytes(corrupt),history_before);
        ASSERT_EQ(native::Evaluate(nr,expected,t3_port_test::Native(in),wrong),nt::Status::kSuccess);
        EXPECT_EQ(wrong.points,n.points);EXPECT_EQ(ForceDifference(wrong.shell,n.shell),0.);
      }
      max_thickness=std::max(max_thickness,std::abs(n.shell.proposed_history.data().thickness-input.thickness));
      if(motion.step+1==64*motion.refinement)preload=n.points[0][0];
      accepted={a.force.proposed_history,a.proposed_section};expected={n.shell.proposed_history,n.points};
    }
    EXPECT_GT(max_thickness,1.e-6);EXPECT_GT(std::abs(expected.points[0][0]-preload),1.e6);
    EXPECT_GT(std::abs(expected.shell.data().internal_work[0]),1.e-6);
    if(scenario)EXPECT_GT(motion.Angle(motion.time()),1.6);
  }
}
} // namespace layered_law1_force_test
