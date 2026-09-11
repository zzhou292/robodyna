#include "ProjectedFixture.h"
#include "NativeOracle.h"
namespace classification_test {
namespace {
void InstallNativeRegistration(Fixture& f, int ikrem) {
  const auto native = NativeRegister(f.Rigid(ikrem));
  const auto count = f.nodes.size();
  ASSERT_EQ(native.five_blocks.size(), 5*count);
  for (std::size_t n = 0; n < count; ++n)
    f.nodes[n].kinematics = {native.five_blocks[n], native.five_blocks[count+n],
      native.five_blocks[2*count+n], native.five_blocks[3*count+n], native.five_blocks[4*count+n]};
}
void SameNativeObserved(const Fixture& f, const NativeResult& projected, const NativeResult& complete) {
  EXPECT_EQ(projected.irupt,complete.irupt);
  EXPECT_EQ(projected.itf,complete.itf);
  EXPECT_EQ(projected.warnings,complete.warnings);
  EXPECT_EQ(projected.penalties,complete.penalties);
  const auto count = f.nodes.size();
  for (const auto node : f.roles.front().slaves)
    for (std::size_t field = 0; field < 5; ++field)
      EXPECT_EQ(projected.five_blocks[field*count+node],complete.five_blocks[field*count+node]);
}
}
TEST(TiedClassificationProjectionNative, CompleteCallerPreservesSlaveProjectionUnderIndependentMasterRegistration) {
  for (int ikrem : {0,1}) {
    for (bool reverse : {false,true}) {
      auto f = ProjectedFixture();
      if (reverse) std::reverse(f.groups.begin(),f.groups.end());
      const auto projected = NativeClassify(f.Input());
      ClassificationResult value;
      ASSERT_TRUE(Classify(f.Input(),&value));
      Compare(value,projected);
      // Populate the full context from native CHECKRBY/KINSET output directly,
      // without calling production registration to construct oracle input.
      InstallNativeRegistration(f,ikrem);
      ASSERT_FALSE(HasFailure());
      ASSERT_NE(f.nodes[7].kinematics.conditions,0);
      ASSERT_NE(f.nodes[7].kinematics.duplicate_conditions,0);
      const auto complete = NativeClassify(f.Input());
      SameNativeObserved(f,projected,complete);
      ASSERT_TRUE(Classify(f.Input(),&value));
      Compare(value,complete);
    }
  }
}
TEST(TiedClassificationProjectionNative, LastObservedRigidMemberAndOtherReadsetRolesHaveNativeConsequences) {
  auto baseline = ProjectedFixture();
  const auto projected = NativeClassify(baseline.Input());
  for (unsigned cause = 0; cause < 6; ++cause) {
    SCOPED_TRACE(cause);
    auto f = ProjectedFixture();
    if (cause == 0) {
      f.groups.back().push_back(2);
      InstallNativeRegistration(f,0);
    } else if (cause == 1) {
      f.cyclic.assign(f.nodes.size(),0);
      f.cyclic[2] = 1;
    } else if (cause == 2) {
      f.sections.push_back({100,{2}});
    } else if (cause == 3) {
      f.rbe2.push_back(2);
    } else if (cause == 4) {
      f.rbe3.push_back({2});
    } else {
      f.roles.push_back({101,2,28,{2},{11}});
    }
    const auto complete = NativeClassify(f.Input());
    ASSERT_GE(complete.irupt.size(),3u);
    EXPECT_EQ(projected.irupt[2],0);
    EXPECT_EQ(complete.irupt[2],1);
    ClassificationResult value;
    ASSERT_TRUE(Classify(f.Input(),&value));
    Compare(value,complete);
  }
}
}
