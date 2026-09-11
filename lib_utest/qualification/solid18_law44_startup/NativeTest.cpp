// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#ifdef REAR18_ORIGINAL
#include "OriginalFixture.h"
#endif

namespace rear_startup_test {
void CheckNative(const law::Reference& reference, const law::Material& material,
    s::Vec3 velocity, unsigned steps) {
  auto expected = NativeInitialize(material,reference.input(),velocity);
  ASSERT_EQ(expected.status,0);
  law::ForceTrial actual;
  ASSERT_EQ(law::InitializeForce(reference,material,velocity,actual),s::Status::Success);
  ASSERT_TRUE(Agree(actual,expected));
  EXPECT_EQ(actual.proposed_history.stamp().sample_index,0u);
  EXPECT_EQ(actual.proposed_history.stamp().time_s,0);
  for (unsigned step = 0; step < steps; ++step) {
    SCOPED_TRACE(step);
    auto interval = Path(reference,step,true);
    interval.base_time_s = actual.proposed_history.stamp().time_s;
    expected = Native(material,expected.next,interval);
    ASSERT_EQ(expected.status,0);
    ASSERT_EQ(law::EvaluateForce(reference,actual.proposed_history,interval,material,actual),s::Status::Success);
    ASSERT_TRUE(Agree(actual,expected));
    EXPECT_EQ(actual.proposed_history.stamp().sample_index,step+1);
  }
}
TEST(Rear18StartupNative, IndependentVirginFieldsThenCarriedEightPointRecurrence) {
  for (bool collapsed : {false,true})
    for (const s::Vec3 velocity : {s::Vec3{},s::Vec3{11.123,-.37,.129}}) {
      CheckNative(Reference(collapsed),Material(),velocity,32);
      ASSERT_FALSE(HasFatalFailure());
    }
}
TEST(Rear18StartupNative, OrdinaryZeroStepAndBadConstructorVelocityRemainRejected) {
  const auto reference = Reference(true);
  const auto material = Material();
  const auto native = NativeInitialize(material,reference.input(),{});
  ASSERT_EQ(native.status,0);
  auto interval = Step(reference,law::History{});
  interval.dt_s = 0;
  EXPECT_NE(Native(material,native.next,interval).status,0);
  EXPECT_NE(NativeInitialize(material,reference.input(),{0,0,INFINITY}).status,0);
  ASSERT_EQ(NativeInitialize(material,reference.input(),{}).status,0);
}
#ifdef REAR18_ORIGINAL
TEST(Rear18StartupSource, All306OriginalTT0FieldsAndThreePositiveIntervals) {
  unsigned collapsed = 0;
  for (unsigned row = 0; row < std::size(rear18_test::original::Cells); ++row) {
    const auto input = rear18_test::original::Input(row);
    SCOPED_TRACE(input.source_element_id);
    law::Reference reference;
    ASSERT_EQ(law::InitializeReference(input,reference),s::Status::Success);
    const auto material = rear_force_test::OriginalParameters(input.source_part_id);
    CheckNative(reference,material,{11.123,-.37,.129},3);
    ASSERT_FALSE(HasFatalFailure());
    collapsed += law::detail::NativeDegeneracy(reference) != 0;
  }
  EXPECT_EQ(std::size(rear18_test::original::Cells),306u);
  EXPECT_EQ(collapsed,109u);
}
#endif
}  // namespace rear_startup_test
