// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
using namespace slot_test;
TEST(OrderedSolidSlots, NativeSlotOrderAndNoRotationAccess) { OrderAndUntouchedRotation(Execute); }
TEST(OrderedSolidSlots, PositiveStiffnessSlotOrderAndAtomicFailure) { StiffnessOrderAndRollback(Execute); }
TEST(OrderedSolidSlots, ForceLateFailuresPreserveAllDestinationsAndRetry) { ForceFailureAndRetry(Execute); }
TEST(OrderedSolidSlots, DistinctSlotsMatchStrictExistingScatterBothSigns) {
  for(int sign:{-1,1}) {
    auto p=Initial();p.sign=sign;
    for(unsigned n=0;n<8;++n) {p.nodes[n]=n;p.force[n]={.3*n,-.1*n,2.7*n};}
    auto expected=p;
    ASSERT_EQ(tl::fea::AccumulateNodalTranslationalForces<8>(expected.nodes,expected.force,
        View(expected),sign),Status::Success);
    Execute(p);ASSERT_EQ(p.status,Status::Success);
    EXPECT_EQ(Bytes(p.destination),Bytes(expected.destination));
  }
}
TEST(OrderedSolidSlots, InvalidPointersAndAliasRejectBeforeWrites) {
  auto p=Initial();const auto before=Bytes(p.destination);auto view=View(p);
  view.force_y=view.force_x;
  EXPECT_EQ(tl::fea::AccumulateRepeatedNodalTranslationalForces<8>(p.nodes,p.force,view),Status::InvalidView);
  EXPECT_EQ(tl::fea::AccumulateRepeatedNodalTranslationalForces<8>(nullptr,p.force,View(p)),Status::InvalidView);
  EXPECT_EQ(tl::fea::AccumulateRepeatedNodalTranslationalForces<8>(p.nodes,nullptr,View(p)),Status::InvalidView);
  EXPECT_EQ(Bytes(p.destination),before);
  const auto scalar=Bytes(p.stiffness);
  EXPECT_EQ(tl::fea::AccumulateRepeatedNodalStiffness<8>(nullptr,p.increments,p.stiffness,9),Status::InvalidView);
  EXPECT_EQ(tl::fea::AccumulateRepeatedNodalStiffness<8>(p.nodes,nullptr,p.stiffness,9),Status::InvalidView);
  EXPECT_EQ(tl::fea::AccumulateRepeatedNodalStiffness<8>(p.nodes,p.increments,nullptr,9),Status::InvalidView);
  EXPECT_EQ(Bytes(p.stiffness),scalar);
}
