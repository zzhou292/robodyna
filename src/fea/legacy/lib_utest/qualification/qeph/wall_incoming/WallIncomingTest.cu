#include "WallIncomingFixture.h"

namespace qeph_wall_incoming_test {
using WallIncomingCuda=tl_test::nodal_temporal::NodalTemporalCuda;
TEST_F(WallIncomingCuda, OneCellIncomingEntryUsesAcceptedBaseContactAndScreenedStep) {
  ScreenBinding binding; std::string error;
  ASSERT_TRUE(ReadScreenBinding(binding,error))<<error;
  for(unsigned fault=0;fault<4;++fault) {
    SCOPED_TRACE(fault);
    auto invalid=binding;
    if(fault==0) invalid.selected_h=0;
    if(fault==1) invalid.selected_h=H0/4;
    if(fault==2) invalid.decision_sha256.clear();
    if(fault==3) invalid.decision_sha256[0]='G';
    IncomingRig rejected(1,invalid); EXPECT_FALSE(rejected.Initialize(1));
    EXPECT_EQ(rejected.coupled.shell.owner.allocations().device_allocations,0u);
    EXPECT_EQ(rejected.coupled.shell.batch.allocations().device_allocations,0u);
    EXPECT_EQ(rejected.coupled.wall.allocations().device_allocations,0u);
  }
  ASSERT_NO_FATAL_FAILURE(IncomingPrefix(1,binding));
}
TEST_F(WallIncomingCuda, SharedTwoCellIncomingEntryKeepsNativeCacheAndContactLedgers) {
  ScreenBinding binding; std::string error;
  ASSERT_TRUE(ReadScreenBinding(binding,error))<<error;
  ASSERT_NO_FATAL_FAILURE(IncomingPrefix(2,binding));
}
TEST_F(WallIncomingCuda, EntryRejectionsPreserveOwnerHistoryK0ContactAndEventsThenRetry) {
  ScreenBinding binding; std::string error;
  ASSERT_TRUE(ReadScreenBinding(binding,error))<<error;
  ASSERT_NO_FATAL_FAILURE(IncomingFailures(binding));
}
} // namespace qeph_wall_incoming_test
