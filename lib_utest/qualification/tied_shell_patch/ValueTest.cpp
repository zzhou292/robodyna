#include "Fixture.h"

namespace tied_patch_test {
TEST(TiedPatchValues,ForceMomentBalanceAndVirtualPowerForWarpedFlatTriangleAndPlacedMasters) {
  for (unsigned shape = 0; shape < 4; ++shape) {
    SCOPED_TRACE(shape);
    const auto input = Geometry(shape);
    tie::Patch patch;
    ASSERT_EQ(tie::PreparePatch(input,patch),tie::Status::Success);
    for (unsigned step = 0; step < 12; ++step) {
      const auto load = Load(step);
      const auto motion = Motion(step,shape == 1);
      tie::MasterLoads master;
      tie::SecondaryMotion secondary;
      ASSERT_EQ(tie::TransferLoad(patch,load,master),tie::Status::Success);
      ASSERT_EQ(tie::RecoverMotion(patch,motion,secondary),tie::Status::Success);
      Vec3 resultant{}, moment{};
      double power = 0;
      for (unsigned i = 0; i < 4; ++i) {
        resultant = math::Add(resultant,master.force[i]);
        moment = math::Add(moment,math::Cross(input.master_position[i],master.force[i]));
        power += math::Dot(master.force[i],motion.velocity[i]);
      }
      Near(resultant,load.force);
      Near(moment,math::Add(load.couple,math::Cross(input.secondary_position,load.force)));
      Near(power,math::Dot(load.force,secondary.velocity)+math::Dot(load.couple,secondary.angular_velocity));
    }
  }
}

TEST(TiedPatchValues,RigidTranslationRotationAndNativeAccelerationPacketAreReproduced) {
  const Vec3 translation{1.1,-.3,4.2}, spin{.7,1.9,-.8};
  const Vec3 acceleration{3.,-7.,11.}, angular_acceleration{-.3,1.2,2.3};
  for (unsigned shape = 0; shape < 4; ++shape) {
    const auto input = Geometry(shape);
    tie::Patch patch;
    ASSERT_EQ(tie::PreparePatch(input,patch),tie::Status::Success);
    tie::MasterMotion motion;
    for (unsigned i = 0; i < 4; ++i) {
      motion.velocity[i] = math::Add(translation,math::Cross(spin,input.master_position[i]));
      motion.acceleration[i] = math::Add(acceleration,math::Cross(angular_acceleration,input.master_position[i]));
    }
    tie::SecondaryMotion output;
    ASSERT_EQ(tie::RecoverMotion(patch,motion,output),tie::Status::Success);
    Near(output.angular_velocity,spin);
    Near(output.velocity,math::Add(translation,math::Cross(spin,input.secondary_position)));
    Near(output.angular_acceleration,angular_acceleration);
    Near(output.acceleration,math::Add(acceleration,math::Cross(angular_acceleration,input.secondary_position)));
  }
}

TEST(TiedPatchValues,RepeatedTriangleSlotRetainsNativeQuarterWeightAndRepeatedForceContribution) {
  const auto input = Geometry(1);
  tie::Patch patch;
  ASSERT_EQ(tie::PreparePatch(input,patch),tie::Status::Success);
  tie::MasterLoads output;
  ASSERT_EQ(tie::TransferLoad(patch,Load(2),output),tie::Status::Success);
  EXPECT_EQ(Bytes(output.force[2]),Bytes(output.force[3]));
  const auto center = patch.values().center;
  const auto unique_mean = math::Scale(math::Add(math::Add(input.master_position[0],
      input.master_position[1]),input.master_position[2]),1./3.);
  EXPECT_GT(math::Norm(math::Subtract(center,unique_mean)),1e-4);
}

TEST(TiedPatchValues,SingularGeometryAndLateArithmeticRejectWithoutReplacingOutputs) {
  tie::Patch patch;
  ASSERT_EQ(tie::PreparePatch(Geometry(0),patch),tie::Status::Success);
  const auto accepted = Bytes(patch);
  auto bad = Geometry(0);
  for (unsigned i = 0; i < 4; ++i) bad.master_position[i] = {double(i),0,0};
  EXPECT_EQ(tie::PreparePatch(bad,patch),tie::Status::SingularPatch);
  EXPECT_EQ(Bytes(patch),accepted);
  bad = Geometry(0);
  bad.master_position[3].z = std::numeric_limits<double>::infinity();
  EXPECT_EQ(tie::PreparePatch(bad,patch),tie::Status::InvalidInput);
  EXPECT_EQ(Bytes(patch),accepted);
  tie::MasterLoads loads;
  ASSERT_EQ(tie::TransferLoad(patch,Load(1),loads),tie::Status::Success);
  const auto saved_loads = Bytes(loads);
  auto bad_load = Load(1);
  bad_load.couple.z = std::numeric_limits<double>::max();
  EXPECT_EQ(tie::TransferLoad(patch,bad_load,loads),tie::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(loads),saved_loads);
  tie::SecondaryMotion motion;
  ASSERT_EQ(tie::RecoverMotion(patch,Motion(2),motion),tie::Status::Success);
  const auto saved_motion = Bytes(motion);
  auto bad_motion = Motion(2);
  bad_motion.acceleration[3].z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(tie::RecoverMotion(patch,bad_motion,motion),tie::Status::InvalidInput);
  EXPECT_EQ(Bytes(motion),saved_motion);
  ASSERT_EQ(tie::RecoverMotion(patch,Motion(2),motion),tie::Status::Success);
  EXPECT_EQ(Bytes(motion),saved_motion);
}
} // namespace tied_patch_test
