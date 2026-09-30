// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "FrozenMotion.h"
#include "../cin_parallel_ordinary/Fault.h"
#include <gtest/gtest.h>

namespace tl::fea::cin_group_test {
TEST(CinParallelGroupsFixture, OriginalZeroMassMembersCannotBecomeOrdinaryFreeNodes) {
  packet::Packet wrong(true, false);
  PopulateGroups(wrong, 0);
  const auto node = packet::Nodes-5;
  ASSERT_EQ(wrong.trial[wrong.TailOffset()+node], 0);
  ASSERT_EQ(wrong.member_nodes[node], 0);
  packet::RunSerialHost(wrong.Input());
  EXPECT_EQ(wrong.control.status, NodalStatus::InvalidOutput);
  EXPECT_EQ(wrong.control.node, node);
  // Exact same factory as CUDA: complete frozen serial input/transfer/motion,
  // not merely the group helper, must admit every positive packet first.
  for (bool capture : {false, true}) for (bool screen : {false, true}) {
    for (unsigned count : {0, 2, 65, 129}) {
      SCOPED_TRACE(capture);
      SCOPED_TRACE(screen);
      SCOPED_TRACE(count);
      auto p = MakePacket(count, capture);
      if (screen) p.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
      for (unsigned step = 0; step < 3; ++step) {
        SCOPED_TRACE(step);
        p.Begin(step+1);
        packet::RunSerialHost(p.Input());
        ASSERT_EQ(p.control.status, NodalStatus::Ok) << p.control.node;
        p.Accept();
      }
    }
  }
}
TEST(CinParallelGroupsMotion, FrozenGroupBodyMatchesHalfFullKickCaptureAndZeroCoefficientMembers) {
  for (bool capture : {false, true}) {
    for (unsigned count : {2, 65, 129}) {
      packet::Packet a(true, capture);
      if (count != 2) PopulateGroups(a, count);
      for (unsigned step = 0; step < 3; ++step) {
        a.Begin(step+1);
        auto b = a;
        for (unsigned g = 0; g < a.groups.size(); ++g) FrozenMotion(a.Input(), g);
        for (std::size_t g = b.groups.size(); g-- > 0;) {
          const auto report = cin_advance::groups::AdvanceGroup(b.Input(), g);
          ASSERT_EQ(report.status, NodalStatus::Ok) << g;
        }
        ASSERT_EQ(a.control.status, NodalStatus::Ok);
        packet::SameSuccessfulPacket(a, b);
        a.Accept();
      }
    }
  }
}
TEST(CinParallelGroupsMotion, LateOrientationAndCandidateFailuresPreserveExactGroupPhase) {
  for (unsigned fault = 0; fault < 3; ++fault) {
    packet::Packet a(true, true);
    ReverseGroups(a);
    const unsigned group = 1;
    const unsigned first = a.members[a.groups[group].offset].node;
    const unsigned last = a.members[a.groups[group].offset+a.groups[group].count-1].node;
    if (fault == 0) a.accepted[9*packet::Nodes+4*last] = 0;
    if (fault == 1) a.groups[group].mass = 0;
    if (fault == 2) a.loads[3*packet::Nodes+last] = 1e12;
    a.Begin(1);
    auto b = a;
    const auto accepted = a.accepted;
    FrozenMotion(a.Input(), group);
    const auto report = cin_advance::groups::AdvanceGroup(b.Input(), group);
    ASSERT_NE(a.control.status, NodalStatus::Ok);
    EXPECT_EQ(report.status, a.control.status);
    EXPECT_EQ(report.last_node, a.control.node);
    EXPECT_EQ(a.control.node, fault == 0 ? last : first);
    packet::SameDoubles(a.trial, b.trial);
    packet::SameDoubles(a.capture, b.capture);
    packet::SameDoubles(a.accepted, accepted);
    packet::SameDoubles(b.accepted, accepted);
    // A clean retry reuses the report object and cannot retain failure fields.
    b = packet::Packet(true, true);
    const auto retry = cin_advance::groups::AdvanceGroup(b.Input(), group);
    EXPECT_EQ(retry.status, NodalStatus::Ok);
    EXPECT_EQ(retry.last_node, UINT32_MAX);
  }
}
} // namespace tl::fea::cin_group_test
