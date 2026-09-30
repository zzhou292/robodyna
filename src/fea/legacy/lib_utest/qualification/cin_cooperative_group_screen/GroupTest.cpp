// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/solvers/cin_limiter/Capture.h"
#include "Fixture.h"

namespace tl::fea::cooperative_test {
TEST(CinCooperativeGroups, MultiTileExactCurrentFrameAndReversedSourceOrder) {
  for (unsigned count : {2, 3, 63, 64, 65, 127, 128, 129, 260}) {
    auto p = Members({count});
    for (auto previous : {0., 1e-6, .125}) {
      p.durations.previous_drift_dt = previous;
      Compare(p);
      std::reverse(p.members.begin(), p.members.end());
      Compare(p);
    }
    std::fill(p.work.begin(), p.work.begin()+2*packet::Nodes, -0.);
    Compare(p);
  }
  auto p = Members({65, 129, 2});
  cin_group_test::ReverseGroups(p);
  Compare(p);
  // Raw overlap is not a new admission profile; it must not race shared staging.
  p.groups[1].offset = p.groups[0].offset;
  p.groups[1].count = p.groups[0].count;
  Compare(p);
}
TEST(CinCooperativeGroups, FreshLateFailureEarlierOverflowMalformedAndRetry) {
  for (unsigned fault = 0; fault < 12; ++fault) {
    auto p = Members({129, 3});
    const auto first = p.members[0].node;
    const auto late = p.members[128].node;
    if (fault == 0) p.groups[0].count = 1;
    if (fault == 1) p.groups[1].count = 1;
    if (fault == 2) p.groups[0].offset = UINT32_MAX;
    if (fault == 3) p.groups[1].count = UINT32_MAX;
    if (fault == 4) p.groups[0].mass = 0;
    if (fault == 5) p.members[128].node = packet::Nodes;
    if (fault == 6) p.dependent[late] = 1;
    if (fault == 7) p.member_nodes[late] = 0;
    if (fault == 8) p.accepted[3*late] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 9) {
      p.work[first] = std::numeric_limits<double>::max();
      p.accepted[3*late] = std::numeric_limits<double>::quiet_NaN();
    }
    if (fault == 10) {
      p.groups[0].mass = 0;
      p.work[12] = -1;
    }
    if (fault == 11) p.work[packet::Nodes+first] = std::numeric_limits<double>::max();
    Compare(p, false);
    auto retry = Members({129, 3});
    retry.Begin(2);
    Compare(retry);
  }
}
TEST(CinCooperativeGroups, StoppedPacketNeverDereferencesLaterInputAndFixedSharedBudget) {
  auto p = Members({65});
  const auto source = screen::Sources(p.Input());
  auto state = staged::BeginGroup(source, 0);
  const auto body = staged::ReadBody(state.body);
  state.stopped = true;
  staged::FoldMember({}, body, UINT32_MAX, {}, state);
  EXPECT_TRUE(state.stopped);
  const auto before = state.report;
  staged::FinishGroup(.8, state);
  Same(before, state.report);
  EXPECT_EQ(sizeof(staged::Tile), 4776u);
  EXPECT_EQ(sizeof(staged::MemberResponses), 72u);
  EXPECT_EQ(staged::Threads, 64u);
}
} // namespace tl::fea::cooperative_test
