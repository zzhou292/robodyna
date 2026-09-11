// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <limits>

namespace rigid_dependent_test {
TEST(RigidDependentStep, ZeroSourceCoefficientsCarryPrimaryMotionAndRealReactions) {
  for(unsigned count:{2u,4u}) {
    const auto in=Initial();Trial out;
    ASSERT_EQ(Evaluate(in,count,out),r::StepStatus::Success);
    for(unsigned k=0;k<count;++k)for(unsigned a=0;a<3;++a) {
      const auto& m=in.member[k];const auto& v=out.member[k];
      EXPECT_DOUBLE_EQ(rigid_test::Get(v.reaction_couple,a),
          m.inertia*rigid_test::Get(v.angular_acceleration,a)-rigid_test::Get(m.couple,a));
      EXPECT_DOUBLE_EQ(rigid_test::Get(v.reaction_force,a),
          m.mass*rigid_test::Get(v.acceleration,a)-rigid_test::Get(m.force,a));
      EXPECT_NEAR(rigid_test::Get(v.omega,a),rigid_test::Get(out.primary.omega,a),2e-15);
    }
    EXPECT_NE(out.member[0].omega.x,0);
    EXPECT_EQ(Evaluate(in,count,out,r::MemberCoefficientPolicy::PositiveIndependent),r::StepStatus::InvalidInput);
  }
}
TEST(RigidDependentStep, LateInvalidCoefficientsPreservePacketAndExactRetry) {
  for(unsigned count:{2u,4u})for(unsigned failure=0;failure<4;++failure) {
    const auto good=Initial();auto bad=good;Trial expected;
    ASSERT_EQ(Evaluate(good,count,expected),r::StepStatus::Success);
    switch(failure) {
      case 0:bad.member[count-1].mass=-1;break;
      case 1:bad.member[count-1].inertia=-1;break;
      case 2:bad.member[count-1].inertia=std::numeric_limits<double>::quiet_NaN();break;
      default:bad.member[count-1].couple.x=std::numeric_limits<double>::infinity();break;
    }
    Trial out=expected;const auto bytes=rigid_step_test::Bytes(out);
    EXPECT_NE(Evaluate(bad,count,out),r::StepStatus::Success);
    EXPECT_EQ(rigid_step_test::Bytes(out),bytes);
    EXPECT_EQ(Evaluate(good,count,out,static_cast<r::MemberCoefficientPolicy>(9)),r::StepStatus::InvalidInput);
    ASSERT_EQ(Evaluate(good,count,out),r::StepStatus::Success);
    EXPECT_EQ(rigid_step_test::Bytes(out),bytes);
  }
}
} // namespace rigid_dependent_test
