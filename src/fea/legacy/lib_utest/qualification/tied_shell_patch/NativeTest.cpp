#include "NativeOracle.h"

namespace tied_patch_test {
TEST(TiedPatchNative,CompleteForceFragmentAndMotionRoutineAgreeAcrossCurrentShapesAndLoads) {
  for (unsigned shape = 0; shape < 4; ++shape) {
    SCOPED_TRACE(shape);
    const auto input = Geometry(shape);
    tie::Patch patch;
    ASSERT_EQ(tie::PreparePatch(input,patch),tie::Status::Success);
    for (unsigned step = 0; step < 16; ++step) {
      SCOPED_TRACE(step);
      tie::MasterLoads loads;
      tie::SecondaryMotion motion;
      ASSERT_EQ(tie::TransferLoad(patch,Load(step),loads),tie::Status::Success);
      ASSERT_EQ(tie::RecoverMotion(patch,Motion(step,shape == 1),motion),tie::Status::Success);
      Agreement(patch,loads,motion,Native(input,Load(step),Motion(step,shape == 1),shape == 1));
    }
  }
}
TEST(TiedPatchNative,PureCoupleAndOffSurfaceForceBothProduceNativeDistributedMoment) {
  const auto input = Geometry(0);
  tie::Patch patch;
  ASSERT_EQ(tie::PreparePatch(input,patch),tie::Status::Success);
  for (const auto& load : {tie::SecondaryLoad{{0,0,0},{.7,-.3,1.1}},
                           tie::SecondaryLoad{{83,-19,27},{0,0,0}}}) {
    tie::MasterLoads forces;
    tie::SecondaryMotion motion;
    ASSERT_EQ(tie::TransferLoad(patch,load,forces),tie::Status::Success);
    ASSERT_EQ(tie::RecoverMotion(patch,Motion(4),motion),tie::Status::Success);
    Agreement(patch,forces,motion,Native(input,load,Motion(4)));
    EXPECT_GT(math::Norm(math::Subtract(forces.force[0],math::Scale(load.force,.25))),.1);
  }
}
TEST(TiedPatchNative,TriangleOracleUsesRepeatedNodeIndexAndIgnoresUnusedFourthNodeMotion) {
  const auto input = Geometry(1);
  const auto physical = Motion(5,true);
  auto unused_node = physical;
  unused_node.velocity[3] = {127,83,-99};
  unused_node.acceleration[3] = {-237,66,22};
  const auto expected = Native(input,Load(5),physical,true);
  EXPECT_EQ(Native(input,Load(5),unused_node,true).values,expected.values);
  EXPECT_NE(Native(input,Load(5),unused_node,false).values,expected.values);
}
} // namespace tied_patch_test
