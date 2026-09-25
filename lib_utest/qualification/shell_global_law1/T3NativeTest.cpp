#include "Fixture.h"
namespace global_law1_test {
TEST(GlobalLaw1Native,T3BothThicknessModesLoadedHeldRotatingReversed) {
  for(int ithk:{0,1})for(unsigned scenario=0;scenario<3;++scenario) {
    auto input=Triangle();t::ReferenceData r;nt::Reference nr;
    ASSERT_EQ(t::InitializeReference(input,r),t::Status::kSuccess);
    ASSERT_EQ(nt::Initialize(t3_port_test::Native(input),nr),nt::Status::kSuccess);
    auto accepted=THistory(r);auto expected=t3_force_port_test::Native(nr,accepted);
    path::Path motion{scenario!=0,scenario==2?2u:1u};motion.angular_speed=18000;
    Profile profile{ithk?Thickness::Accepted:Thickness::Reference,1.};
    double thickness_change=0;
    for(;motion.step<160*motion.refinement;++motion.step) {
      SCOPED_TRACE(ithk);SCOPED_TRACE(scenario);SCOPED_TRACE(motion.step);
      const auto in=motion.Interval(input);t::ForceTrial a;nt::ForceTrial b;
      ASSERT_EQ(t::EvaluateGlobalLaw1Force(profile,r,accepted,in,a),t::Status::kSuccess);
      ASSERT_EQ(global::Evaluate(nr,expected,t3_port_test::Native(in),ithk,b),nt::Status::kSuccess);
      t3_force_port_test::Agreement(r,in,a,b,path::cv::Tolerance);
      ASSERT_FALSE(::testing::Test::HasFailure());
      thickness_change=std::max(thickness_change,std::abs(b.proposed_history.data().thickness-input.thickness));
      accepted=a.proposed_history;expected=b.proposed_history;
    }
    EXPECT_GT(thickness_change,1.e-6);
  }
}
} // namespace global_law1_test
