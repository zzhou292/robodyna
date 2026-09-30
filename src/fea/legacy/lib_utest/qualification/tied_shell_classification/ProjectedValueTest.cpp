#include "ProjectedFixture.h"
namespace classification_test {
TEST(TiedClassificationProjectionValues, NonobservedRigidOrdersAndKindsLeaveObservedSlaveStateAndDecodeTableUnchanged) {
  for (int ikrem : {0,1}) {
    for (bool reverse : {false,true}) {
      auto f = ProjectedFixture();
      if (reverse) std::reverse(f.groups.begin(),f.groups.end());
      ClassificationResult projected, registered, complete;
      ASSERT_TRUE(Classify(f.Input(),&projected));
      ASSERT_TRUE(RegisterRigidMembers(f.Rigid(ikrem),&registered));
      auto input = f.Input();
      input.context.nodes = registered.nodes();
      ASSERT_TRUE(Classify(input,&complete));
      SameObserved(projected,complete);
      EXPECT_NE(complete.nodes().data[7].kinematics.conditions,0);
      EXPECT_NE(complete.nodes().data[7].kinematics.duplicate_conditions,0);
      EXPECT_EQ(projected.nodes().data[7].kinematics.conditions,0);
    }
  }
}
TEST(TiedClassificationProjectionValues, LateSlaveInfluenceBreaksProjectionAndMustNotBeDroppedBySourceReceipt) {
  auto f = ProjectedFixture();
  ClassificationResult projected, registered, complete;
  ASSERT_TRUE(Classify(f.Input(),&projected));
  f.groups.back().push_back(2);
  ASSERT_TRUE(RegisterRigidMembers(f.Rigid(),&registered));
  auto input = f.Input();
  input.context.nodes = registered.nodes();
  ASSERT_TRUE(Classify(input,&complete));
  EXPECT_EQ(projected.irupt().data[2],0);
  EXPECT_EQ(complete.irupt().data[2],1);
  EXPECT_NE(projected.nodes().data[2].kinematics.conditions,complete.nodes().data[2].kinematics.conditions);
}
}
