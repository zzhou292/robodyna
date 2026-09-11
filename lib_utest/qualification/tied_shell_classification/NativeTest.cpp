// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"

namespace classification_test {
namespace {
void Check(Fixture& f) {
  const auto input=f.Input();
  ClassificationResult out;
  ASSERT_TRUE(Classify(input,&out));
  Compare(out,NativeClassify(input));
  std::size_t offset=0;
  for(std::size_t n=0;n<input.interfaces.count;++n) {
    const auto& role=input.interfaces.data[n];
    for(std::size_t i=0;i<role.slaves.count;++i) {
      ASSERT_EQ(out.slave_nodes().data[offset+i],role.slaves.data[i]);
      ASSERT_EQ(out.nodes().data[out.slave_nodes().data[offset+i]].source_id,
                input.context.nodes.data[role.slaves.data[i]].source_id);
    }
    offset+=role.slaves.count;
  }
}
}
TEST(TiedClassificationNative,AllConditionKindsWallAndGlobalDecodeMutation) {
  for(const int kind:{0,1,2,4,8,128,16,32,64,256,512,1024,2048,4096}) {
    SCOPED_TRACE(kind);
    Fixture f;
    f.nodes[0].kinematics.conditions=kind;
    if(kind==4) f.nodes[0].kinematics.translation=11;
    f.roles[0].slaves={0,1};
    Check(f);
    f.roles[0].slaves={1,0};
    Check(f);
  }
  Fixture f;
  f.nodes[0].kinematics={2,7,7,2,4};
  f.nodes[1].kinematics={0,7,7,0,4};
  f.roles[0].slaves={0,1};
  Check(f);
}
TEST(TiedClassificationNative,GlobalRolesAndEverySelectedMaskPass) {
  Fixture f;
  f.roles={{100,2,28,{0,1,2,3,4,5,6,7},{10}},
           {101,2,27,{0},{11}},{102,2,0,{}, {1}},{103,25,28,{8},{9}}};
  f.sections={{100,{2}},{101,{3}},{99,{7}}};
  f.cyclic.assign(f.nodes.size(),0); f.cyclic[4]=3;
  f.rbe2={5}; f.rbe3={{6}};
  Check(f);
  for(const int level:{0,1,2,25,26,27,28}) {
    f.roles[2].level=level;
    Check(f);
  }
  f.roles={{100,2,28,{0,0},{11}}};
  Check(f);
}
TEST(TiedClassificationNative,TetraBothCornerChecksAndMasterPropagation) {
  Fixture f;
  f.tetra={{0,1,2},{3,4,5}};
  f.tags.assign(f.nodes.size(),0); f.tags[0]=-1; f.tags[3]=2;
  f.roles[0].slaves={0,1,2,3,4,5};
  Check(f);
  f.roles[0].slaves={0,1,3,5};
  Check(f);
  f.roles[0].slaves={1,2,4,5}; f.roles[0].masters={0,3};
  Check(f);
  f.roles.push_back({101,2,28,{8},{0,3}});
  Check(f);
}
TEST(TiedClassificationNative,OriginalRigidLoopSuppliedMembersAndScratchResets) {
  Fixture f;
  f.groups={{0,1},{0,2,2},{3}};
  f.nodes[3].kinematics={4,11,0,0,0};
  for(const int ikrem:{0,1}) for(const int level:{0,1}) {
    const auto input=f.Rigid(ikrem,level);
    ClassificationResult out;
    ASSERT_TRUE(RegisterRigidMembers(input,&out));
    Compare(out,NativeRegister(input));
    auto tagging=f.Input(); tagging.context.nodes=out.nodes();
    ClassificationResult tags;
    ASSERT_TRUE(Classify(tagging,&tags));
    Compare(tags,NativeClassify(tagging));
  }
  // Re-entering CHECKRBY is a new private scratch scope. It still observes the
  // prior global DOF registration; compare every duplicate/conflict block.
  ClassificationResult first,second;
  ASSERT_TRUE(RegisterRigidMembers(f.Rigid(),&first));
  auto input=f.Rigid(); input.context.nodes=first.nodes();
  ASSERT_TRUE(RegisterRigidMembers(input,&second));
  Compare(second,NativeRegister(input));
}
TEST(TiedClassificationNative,LateRejectionLeavesOldValueAndExactNativeRetry) {
  Fixture f;
  f.roles[0].slaves={0,1}; f.nodes[0].kinematics.conditions=2;
  ClassificationResult out;
  ASSERT_TRUE(Classify(f.Input(),&out));
  const auto saved=out;
  f.rbe3={{2},{99}};
  EXPECT_FALSE(Classify(f.Input(),&out)); Same(out,saved);
  f.rbe3.clear();
  ASSERT_TRUE(Classify(f.Input(),&out)); Same(out,saved);
  Compare(out,NativeClassify(f.Input()));
  auto malformed=f.Input();
  f.nodes.back().kinematics.rotation=19;
  EXPECT_THROW(NativeClassify(malformed),std::invalid_argument);
  f.nodes.back().kinematics.rotation=0;
  Compare(out,NativeClassify(f.Input()));
}
} // namespace classification_test
