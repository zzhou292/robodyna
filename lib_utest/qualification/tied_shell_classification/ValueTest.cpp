// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"

namespace classification_test {
TEST(TiedClassification,EveryPenaltyKindAndWallOnlyRemainDistinct) {
  for(const int kind:{1,2,8,128,16,32,64,256,512,1024,2048,4096}) {
    Fixture f;
    f.nodes[0].kinematics.conditions=kind;
    ClassificationResult result;
    ASSERT_TRUE(Classify(f.Input(),&result));
    EXPECT_EQ(result.irupt().data[0],1)<<kind;
    EXPECT_EQ(result.native_penalty_warnings(),1);
    EXPECT_EQ(result.nodes().data[0].kinematics.conditions,kind);
  }
  Fixture f;
  f.nodes[0].kinematics={4,11,0,0,0};
  ClassificationResult result;
  ASSERT_TRUE(Classify(f.Input(),&result));
  EXPECT_EQ(result.irupt().data[0],0);
  EXPECT_EQ(result.phase(),ClassificationPhase::InterfaceTaggedBeforeKinChk);
  const auto k=result.nodes().data[0].kinematics;
  EXPECT_EQ(k.conditions,6); EXPECT_EQ(k.translation,17); EXPECT_EQ(k.rotation,7);
  EXPECT_EQ(k.incompatible_conditions,2); EXPECT_EQ(result.native_kinset_warnings(),3);
}
TEST(TiedClassification,GlobalItfMutationUsesSourceOrderAndFreshTables) {
  Fixture f;
  f.nodes[0].kinematics.conditions=2;
  f.roles[0].slaves={0,1};
  ClassificationResult first,retry,reversed;
  ASSERT_TRUE(Classify(f.Input(),&first));
  EXPECT_EQ(first.irupt().data[0],1); EXPECT_EQ(first.irupt().data[1],0);
  EXPECT_EQ(first.interface_decode().data[2],0);
  EXPECT_EQ(first.nodes().data[1].kinematics.conditions,6);
  ASSERT_TRUE(Classify(f.Input(),&retry));
  Same(first,retry);
  f.roles[0].slaves={1,0};
  ASSERT_TRUE(Classify(f.Input(),&reversed));
  EXPECT_EQ(reversed.nodes().data[1].kinematics.conditions,2);
  EXPECT_EQ(reversed.interface_decode().data[2],0);
}
TEST(TiedClassification,DuplicateInterfacesAndMainRolesAreGlobal) {
  Fixture f;
  f.roles={{100,2,28,{0,1},{10}},{101,2,27,{0},{11}},
           {102,2,0,{}, {1}},{103,25,28,{2},{3}}};
  ClassificationResult result;
  ASSERT_TRUE(Classify(f.Input(),&result));
  ASSERT_EQ(result.irupt().count,4);
  EXPECT_EQ(result.irupt().data[0],1); EXPECT_EQ(result.irupt().data[1],1);
  EXPECT_EQ(result.irupt().data[2],1); EXPECT_EQ(result.irupt().data[3],0);
  EXPECT_FALSE(result.interfaces().data[3].selected);
  EXPECT_EQ(result.nodes().data[2].kinematics.conditions,0);
  f.roles={{100,2,28,{0,0},{11}}};
  ASSERT_TRUE(Classify(f.Input(),&result));
  EXPECT_EQ(result.native_penalty_warnings(),2);
}
TEST(TiedClassification,SectionsCyclicAndRbeRolesUseOnlySelectedBranches) {
  Fixture f;
  f.roles[0].slaves={0,1,2,3,4,5};
  f.sections={{100,{0}},{101,{1}},{99,{5}}};
  f.cyclic.assign(f.nodes.size(),0); f.cyclic[2]=1;
  f.rbe2={3}; f.rbe3={{4}};
  ClassificationResult result;
  ASSERT_TRUE(Classify(f.Input(),&result));
  for(int i=0;i<5;++i) EXPECT_EQ(result.irupt().data[i],1);
  EXPECT_EQ(result.irupt().data[5],0);
}
TEST(TiedClassification,TetraMissingCornerAndPenalizedMidpointPropagate) {
  Fixture f;
  f.tetra={{0,1,2}};
  f.tags.assign(f.nodes.size(),0); f.tags[0]=-1;
  f.roles[0].slaves={0,1,2};
  ClassificationResult result;
  ASSERT_TRUE(Classify(f.Input(),&result));
  EXPECT_EQ(result.native_penalty_warnings(),0);
  f.roles[0].slaves={0,1};
  ASSERT_TRUE(Classify(f.Input(),&result));
  EXPECT_EQ(result.irupt().data[0],1); EXPECT_EQ(result.irupt().data[1],1);
  f.roles[0].slaves={1,2}; f.roles[0].masters={0};
  ASSERT_TRUE(Classify(f.Input(),&result));
  EXPECT_EQ(result.native_penalty_warnings(),2);
}
TEST(TiedClassification,RigidRegistrationKeepsFiveBlocksAndCallerScratchScope) {
  Fixture f;
  f.groups={{0,1},{0,2}};
  ClassificationResult registered;
  ASSERT_TRUE(RegisterRigidMembers(f.Rigid(),&registered));
  EXPECT_EQ(registered.phase(),ClassificationPhase::RigidMembersRegistered);
  EXPECT_EQ(registered.nodes().data[0].kinematics.duplicate_conditions,8);
  EXPECT_EQ(registered.native_kinset_warnings(),6);
  EXPECT_EQ(registered.nodes().data[1].kinematics.duplicate_conditions,0);
  auto input=f.Input(); input.context.nodes=registered.nodes();
  ClassificationResult result;
  ASSERT_TRUE(Classify(input,&result));
  EXPECT_EQ(result.irupt().data[0],1);
  f.roles[0].slaves={3}; f.roles[0].masters={0,1};
  input=f.Input(); input.context.nodes=registered.nodes();
  ASSERT_TRUE(Classify(input,&result));
  EXPECT_EQ(result.irupt().data[0],0);
  ASSERT_TRUE(RegisterRigidMembers(f.Rigid(1),&result));
  EXPECT_EQ(result.nodes().data[0].kinematics.duplicate_conditions,128);
  ASSERT_TRUE(RegisterRigidMembers(f.Rigid(0,1),&result));
  EXPECT_EQ(result.nodes().data[0].kinematics.conditions,0);
}
} // namespace classification_test
