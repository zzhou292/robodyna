// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
namespace physical_activity_test {
TEST(PhysicalActivityCuda, ExactSourceFreshBorrowAndOpaqueReceiptLifecycle) {
  Fixture f; ASSERT_TRUE(f.Initialize());
  fe::PhysicalActivitySnapshot other; auto roster = f.rig.Participants(); roster.solids = nullptr;
  EXPECT_NE(other.Initialize(f.rig.owner, f.rig.publication, f.rig.fixture.physical,
      roster, f.rig.fixture.Identity()).status, Status::Ok);
  ASSERT_TRUE(f.Begin()); ASSERT_TRUE(f.accepted.valid());
  const auto captures = ActivityReadbackCount();
  fe::PhysicalActivityDeviceView a, b;
  ASSERT_TRUE(Good(f.snapshot.BorrowAccepted(f.rig.owner, f.token, f.assembly, f.accepted, &a)));
  ASSERT_TRUE(Good(f.snapshot.BorrowAccepted(f.rig.owner, f.token, f.assembly, f.accepted, &b)));
  EXPECT_EQ(ActivityReadbackCount(), captures);
  EXPECT_EQ(a.generation, b.generation); EXPECT_EQ(a.qeph.base, b.qeph.base);
  EXPECT_EQ(Read(a.qeph.base, 2, a.stream), (std::vector<std::uint8_t>{1,1}));
  EXPECT_EQ(Read(a.t3.base, 1, a.stream), (std::vector<std::uint8_t>{1}));
  auto forged = f.assembly; ++forged.attempt;
  EXPECT_EQ(f.snapshot.BorrowAccepted(f.rig.owner, f.token, forged, f.accepted, &b).status, Status::StaleReceipt);
  EXPECT_EQ(a.generation, b.generation);
  fe::NodalTrialToken foreign;
  EXPECT_NE(f.snapshot.BorrowAccepted(f.rig.owner, foreign, f.assembly, f.accepted, &b).status, Status::Ok);
  ASSERT_TRUE(f.PreparePhysical()); ASSERT_FALSE(f.accepted.valid());
  ASSERT_TRUE(f.CapturePrepared()); ASSERT_TRUE(f.candidate.valid());
  ASSERT_TRUE(Good(f.snapshot.BorrowPrepared(f.rig.owner, f.token, f.common, f.prepared, f.candidate, &b)));
  ComparePreparedReadbacks(f, b);
  EXPECT_EQ(Read(b.qeph.current, 2, b.stream), Read(b.qeph.base, 2, b.stream));
  auto forged_diagnostics = f.common; forged_diagnostics.qbat.active_count = 0;
  EXPECT_NE(f.snapshot.BorrowPrepared(f.rig.owner, f.token, forged_diagnostics, f.prepared, f.candidate, &a).status, Status::Ok);
  ASSERT_TRUE(f.Commit()); EXPECT_FALSE(f.candidate.valid());
  ASSERT_TRUE(f.Begin()); ASSERT_TRUE(f.accepted.valid()); EXPECT_EQ(f.rig.owner.accepted().epoch, 1u);
  f.rig.owner.Discard(); EXPECT_FALSE(f.accepted.valid()); f.rig.publication.DiscardTrial();
}
TEST(PhysicalActivityCuda, DuplicateCaptureAliasAndDestructionCannotGrantAuthority) {
  Fixture f; ASSERT_TRUE(f.Initialize()); ASSERT_TRUE(f.Begin());
  auto held = f.accepted;
  EXPECT_EQ(f.snapshot.CaptureAccepted(f.rig.owner, f.token, f.assembly, &f.accepted).status, Status::StaleReceipt);
  EXPECT_FALSE(held.valid()); EXPECT_FALSE(f.accepted.valid());
  f.Discard(); ASSERT_TRUE(f.Begin());
  auto* alias = reinterpret_cast<fe::PhysicalActivityDeviceView*>(const_cast<fe::NodalCoefficientNode*>(f.rig.fixture.ledger.nodes().data()));
  EXPECT_EQ(f.snapshot.BorrowAccepted(f.rig.owner, f.token, f.assembly, f.accepted, alias).status, Status::InvalidInput);
  ASSERT_TRUE(f.accepted.valid());
  fe::PhysicalAcceptedActivityReceipt expires;
  f.Discard();
  {
    fe::PhysicalActivitySnapshot temporary;
    ASSERT_TRUE(Good(temporary.Initialize(f.rig.owner, f.rig.publication, f.rig.fixture.physical,
        f.rig.Participants(), f.rig.fixture.Identity())));
    ASSERT_TRUE(f.rig.Begin(f.token, f.assembly));
    ASSERT_TRUE(Good(temporary.CaptureAccepted(f.rig.owner, f.token, f.assembly, &expires)));
    EXPECT_TRUE(expires.valid());
  }
  EXPECT_FALSE(expires.valid()); f.Discard();
}
} // namespace physical_activity_test
