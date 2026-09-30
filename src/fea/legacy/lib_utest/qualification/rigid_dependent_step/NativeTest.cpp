// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "../nodal_rigid_group/GroupStepNativeFixture.h"

namespace rigid_dependent_test {
TEST(RigidDependentNative, ZeroMassAndInertiaMatchIndependentFourMemberNativeHistory) {
  auto actual=Initial(),native=actual;
  for(unsigned interval=0;interval<64;++interval) {
    SCOPED_TRACE(interval);
    Trial out;ASSERT_EQ(Evaluate(actual,4,out),r::StepStatus::Success);
    const auto expected=rigid_step_test::NativePacket(native);
    rigid_step_test::Agreement(out,expected);
    Advance(actual,out,4);Advance(native,expected,4);
  }
}
TEST(RigidDependentNative, ZeroInertiaMatchesIndependentTwoMemberFiniteRotationHistory) {
  auto actual=Initial(),native=actual;
  for(unsigned interval=0;interval<64;++interval) {
    SCOPED_TRACE(interval);
    Trial out;ASSERT_EQ(Evaluate(actual,2,out),r::StepStatus::Success);
    const auto expected=rigid_step_test::NativeTwoPacket(native,.001);
    rigid_step_test::Agreement(out,expected);
    Advance(actual,out,2);Advance(native,expected,2);
  }
}
} // namespace rigid_dependent_test
