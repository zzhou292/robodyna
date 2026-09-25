#include "Fixture.h"
namespace global_law1_test {
TEST(GlobalLaw1Native,QephBothThicknessModesLoadedHeldRotatingReversed) {
  for(int ithk:{0,1})for(unsigned scenario=0;scenario<3;++scenario) {
    auto input=Quad();q::ReferenceData r;nq::Reference nr;
    ASSERT_EQ(q::InitializeReference(input,r),q::Status::kSuccess);
    ASSERT_EQ(nq::Initialize(qeph_startup_test::NativeInput(input),nr),nq::Status::kSuccess);
    auto accepted=QHistory(r);auto expected=QNativeHistory(nr,accepted);
    path::Path motion{scenario!=0,scenario==2?2u:1u};motion.angular_speed=18000;
    Profile profile{ithk?Thickness::Accepted:Thickness::Reference,1.};
    double thickness_change=0;
    for(;motion.step<160*motion.refinement;++motion.step) {
      SCOPED_TRACE(ithk);
      SCOPED_TRACE(scenario);
      SCOPED_TRACE(motion.step);
      const auto in=motion.Interval(input);q::ForceTrial a;nq::ForceTrial b;
      ASSERT_EQ(q::EvaluateGlobalLaw1Force(profile,r,accepted,in,a),q::Status::kSuccess);
      ASSERT_EQ(global::Evaluate(nr,expected,qeph_kinematics_test::NativeInterval(in),ithk,b),nq::Status::kSuccess);
      qeph_force_port_test::ForceAgreement(a,b,input,in,path::cv::Tolerance);
      ASSERT_FALSE(::testing::Test::HasFailure());
      thickness_change=std::max(thickness_change,std::abs(b.proposed_history.data().thickness-input.thickness));
      accepted=a.proposed_history;expected=b.proposed_history;
    }
    EXPECT_GT(thickness_change,1.e-6);
  }
}
} // namespace global_law1_test
