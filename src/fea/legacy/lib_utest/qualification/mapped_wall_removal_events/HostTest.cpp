#include "Fixture.h"
namespace wall_removal_test {
TEST(WallRemovalHost, EveryBytePairMatchesOriginalBranchAndFailurePacket) {
  Packet a(1), b(1);
  for (unsigned accepted = 0; accepted < 256; ++accepted) {
    for (unsigned proposed = 0; proposed < 256; ++proposed) {
      a.Reset(); b.Reset();
      a.accepted[0] = b.accepted[0] = accepted;
      a.proposed[0] = b.proposed[0] = proposed;
      Serial(a); Staged(b); Same(a, b);
    }
  }
}
TEST(WallRemovalHost, AllCountsAndPatternsRetainExactSourceOrderedCertificates) {
  for (unsigned n : {0u, 1u, 31u, 32u, 33u, 255u, 256u, 257u, 777u, 524288u}) {
    Packet a(n), b(n);
    for (unsigned pattern = 0; pattern < 4; ++pattern) {
      a.Reset(pattern); b.Reset(pattern);
      Serial(a); Staged(b); Same(a, b);
      ASSERT_EQ(b.storage.control.status, c::NodalWallDeviceStatus::Ok);
      ASSERT_TRUE(b.storage.result.diagnostics.valid);
    }
  }
}
TEST(WallRemovalHost, NumericalFailurePriorityPartialSummaryAndRetry) {
  Packet a, b;
  for (unsigned fault = 0; fault < 13; ++fault) {
    SCOPED_TRACE(fault);
    Fault(a, fault); Fault(b, fault);
    Serial(a); Staged(b); Same(a, b);
    if (fault == 6) {
      EXPECT_EQ(b.storage.control.status, c::NodalWallDeviceStatus::NonFiniteArithmetic);
      EXPECT_LT(b.storage.control.parent, 256u);
    }
    if (fault == 7) EXPECT_EQ(b.storage.control.parent, 254u);
    a.Reset(); b.Reset();
    Serial(a); Staged(b); Same(a, b);
    EXPECT_TRUE(b.storage.result.diagnostics.valid);
  }
}
TEST(WallRemovalHost, NonremovedInvalidAndPriorErrorNeverConsumeParentPayload) {
  for (unsigned mode = 0; mode < 3; ++mode) {
    Packet a(1), b(1);
    a.Reset(1); b.Reset(1);
    a.storage.result.parents = b.storage.result.parents = nullptr;
    if (mode == 1) a.accepted[0] = b.accepted[0] = 255;
    if (mode == 2) a.storage.control.status = b.storage.control.status = c::NodalWallDeviceStatus::PointFailure;
    Serial(a); Staged(b); Same(a, b);
  }
}
TEST(WallRemovalHost, EveryBitAndEmptyWordOverwriteRemainOrdered) {
  Packet a(32), b(32);
  for (unsigned bit = 0; bit < 32; ++bit) {
    a.Reset(1); b.Reset(1);
    a.proposed[bit] = b.proposed[bit] = 0;
    a.parents[bit].potential = b.parents[bit].potential = {0, 0, 0, NAN};
    Serial(a); Staged(b); Same(a, b);
    EXPECT_EQ(r::First(1u << bit), bit);
  }
  EXPECT_EQ(sizeof(r::Tile), 64u);
}
} // namespace wall_removal_test
