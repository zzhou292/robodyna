// SPDX-License-Identifier: AGPL-3.0-or-later
#include "CudaRig.h"

namespace solid_validation_test {
TEST_F(SolidCandidateCuda, FrozenWholeStartupAndCarriedCandidateAllFamiliesBothFoamAndRearProfiles) {
  for (int flag : {1, 2}) for (bool analytic : {false, true}) {
    DeviceRig rig;
    ASSERT_TRUE(rig.Initialize(flag, analytic));
    ASSERT_FALSE(HasFailure());
    for (unsigned epoch = 0; epoch < 8; ++epoch) {
      ASSERT_TRUE(rig.Candidate(epoch, epoch + 1));
      ASSERT_EQ(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::Success);
      EXPECT_TRUE(DeviceRig::Header(rig.parallel).control.diagnostics.valid);
      EXPECT_EQ(DeviceRig::Header(rig.parallel).control.diagnostics.epoch, epoch + 1);
      ASSERT_FALSE(HasFailure());
    }
  }
}
TEST_F(SolidCandidateCuda, FailedTrialLeavesAcceptedSlabUntouchedAndFreshRetryOverwritesAllFlags) {
  DeviceRig rig;
  ASSERT_TRUE(rig.Initialize());
  const auto accepted = rig.serial;
  ASSERT_TRUE(rig.Candidate(0, 1, true));
  EXPECT_NE(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::Success);
  const d::FamilyLayout* families[] {&rig.host.layout.solid18, &rig.host.layout.solid24,
      &rig.host.layout.solid6z, &rig.host.layout.solid18_law44, &rig.host.layout.solid18_law90};
  for (const auto* family : families) {
    EXPECT_EQ(std::memcmp(accepted.data() + family->slab[0].offset,
        rig.parallel.data() + family->slab[0].offset, family->slab[0].bytes), 0);
  }
  ASSERT_TRUE(rig.Candidate(0, 2));
  EXPECT_EQ(DeviceRig::Header(rig.parallel).control.status, s::BatchStatus::Success);
  for (const auto* family : families)
    for (std::size_t p = 0; p < family->result_valid.count; ++p)
      EXPECT_EQ(rig.parallel[family->result_valid.offset + p], 1);
}
} // namespace solid_validation_test
