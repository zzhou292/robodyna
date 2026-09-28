// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace physical_activity_test {
TEST(PhysicalActivityCuda, GenuineRemovalDiscardRetryCommitAndInactiveNextBase) {
  Fixture f(1e-9); ASSERT_TRUE(f.Initialize());
  p::Snapshot before, after; ASSERT_TRUE(f.rig.Read(before));
  for (unsigned retry = 0; retry < 2; ++retry) {
    ASSERT_TRUE(f.Begin());
    const auto node = f.rig.fixture.domain.Find(14);
    const double force = 2*f.rig.fixture.m[node]*.001/(p::H*p::H);
    ASSERT_EQ(cudaMemcpyAsync(f.assembly.forces.force_x + node, &force, sizeof(force),
        cudaMemcpyHostToDevice, f.assembly.stream), cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(f.assembly.stream), cudaSuccess);
    ASSERT_TRUE(f.PreparePhysical()); ASSERT_TRUE(f.CapturePrepared());
    fe::PhysicalActivityDeviceView v;
    ASSERT_TRUE(Good(f.snapshot.BorrowPrepared(f.rig.owner, f.token, f.common, f.prepared, f.candidate, &v)));
    EXPECT_EQ(Read(v.t3.base, 1, v.stream), (std::vector<std::uint8_t>{1}));
    EXPECT_EQ(Read(v.t3.current, 1, v.stream), (std::vector<std::uint8_t>{0}));
    EXPECT_EQ(v.t3.summary.active_count, 0u); EXPECT_EQ(v.t3.summary.first_inactive, 0u);
    EXPECT_EQ(v.t3.summary.removed_count, 1u); EXPECT_EQ(v.t3.summary.first_removed, 0u);
    if (retry == 0) { f.Discard(); ASSERT_TRUE(f.rig.Read(after)); p::Exact(before, after); }
    else ASSERT_TRUE(f.Commit());
  }
  ASSERT_TRUE(f.Begin());
  fe::PhysicalActivityDeviceView v;
  ASSERT_TRUE(Good(f.snapshot.BorrowAccepted(f.rig.owner, f.token, f.assembly, f.accepted, &v)));
  EXPECT_EQ(Read(v.t3.base, 1, v.stream), (std::vector<std::uint8_t>{0}));
  EXPECT_EQ(v.t3.summary.removed_count, 0u); EXPECT_EQ(v.t3.summary.active_count, 0u);
  f.Discard();
}
TEST(PhysicalActivityCuda, PreparedCopyFailureLeavesMasksAndOwnerUnchanged) {
  Fixture f; ASSERT_TRUE(f.Initialize()); ASSERT_TRUE(f.Begin());
  fe::PhysicalActivityDeviceView before;
  ASSERT_TRUE(Good(f.snapshot.BorrowAccepted(f.rig.owner, f.token, f.assembly, f.accepted, &before)));
  const auto bytes = Read(before.qeph.base, 2, before.stream);
  p::Snapshot physical_before, physical_after; ASSERT_TRUE(f.rig.Read(physical_before));
  ASSERT_TRUE(f.PreparePhysical()); ArmCopyFailure();
  EXPECT_EQ(f.snapshot.CapturePrepared(f.rig.owner, f.token, f.common, f.prepared, f.accepted, &f.candidate).status, Status::DeviceFailure);
  EXPECT_FALSE(f.candidate.valid()); EXPECT_FALSE(f.accepted.valid());
  EXPECT_EQ(Read(before.qeph.base, 2, before.stream), bytes);
  f.Discard(); ASSERT_TRUE(f.rig.Read(physical_after)); p::Exact(physical_before, physical_after);
}
} // namespace physical_activity_test
