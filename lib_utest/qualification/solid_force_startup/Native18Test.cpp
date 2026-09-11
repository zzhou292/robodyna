// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../solid18_force/PacketValues.h"
#include "../solid18_reference/SourceFixture.h"
#include "lib_src/elements/solids/ForceStiffness.h"

namespace solid18_force_test {
void CheckStartup(const s::Reference& reference,s::Vec3 velocity,bool recurrence) {
  const auto material=Material();
  auto native=NativeInitial(reference.input());
  s::PrescribedInterval initial;
  for (unsigned n=0;n<8;++n) {
    initial.position_endpoint_m[n]=reference.input().position_m[n];
    initial.velocity_midpoint_m_s[n]=velocity;
  }
  s::ForceTrial actual;
  ASSERT_EQ(s::InitializeForce(reference,material,velocity,actual),s::Status::Success);
  const auto expected=Native(material,native,initial);
  ASSERT_EQ(expected.status,0);
  ASSERT_TRUE(Agree(actual,expected));
  ASSERT_EQ(actual.proposed_history.stamp().sample_index,0u);
  ASSERT_EQ(actual.proposed_history.stamp().time_s,0);
  tl::fea::solids::NodalStiffness stiffness;
  ASSERT_TRUE(tl::fea::solids::PrepareNodalStiffness(actual,stiffness));
  EXPECT_NEAR(stiffness.translation_n_m,.25*expected.diagnostics[4],
              3e-11*stiffness.translation_n_m);
  native=expected.next;
  auto accepted=actual.proposed_history;
  for (unsigned step=0;recurrence && step<4;++step) {
    const auto interval=Path(reference,step);
    const auto next=Native(material,native,interval);
    ASSERT_EQ(s::EvaluateForce(reference,accepted,interval,material,actual),s::Status::Success);
    ASSERT_TRUE(Agree(actual,next));
    accepted=actual.proposed_history;
    native=next.next;
  }
}
TEST(SolidStartupNative, Solid18IndependentTT0ThenPositiveIntervals) {
  for (bool distorted : {false,true}) {
    for (const s::Vec3 velocity : {s::Vec3{},s::Vec3{11.123,-.37,.129}})
      CheckStartup(Reference(distorted),velocity,true);
  }
}
TEST(SolidStartupSource, All908OriginalAdhesiveInitialFieldsAndStiffness) {
  for (unsigned row=0;row<solid18_test::SourceCount;++row) {
    const auto input=solid18_test::Source(row);
    SCOPED_TRACE(input.source_element_id);
    s::Reference reference;
    ASSERT_EQ(s::InitializeReference(input,reference),s::Status::Success);
    CheckStartup(reference,{11.123,0,0},false);
    ASSERT_FALSE(HasFatalFailure());
  }
  EXPECT_EQ(solid18_test::SourceCount,908u);
}
} // namespace solid18_force_test
