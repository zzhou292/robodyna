// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fault.h"
#include <gtest/gtest.h>
#include <cmath>

namespace tl::fea::cin_parallel_test {
TEST(CinParallelSerialFixture, CompleteSuccessPacketsCoverBothTemporalPhasesAndPhysicalRoles) {
  for (bool groups : {false, true}) {
    for (bool capture : {false, true}) {
      for (bool screen : {false, true}) {
        SCOPED_TRACE(groups);
        SCOPED_TRACE(capture);
        SCOPED_TRACE(screen);
        Packet packet(groups, capture);
        const auto original_loads = packet.loads;
        if (screen) packet.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
        for (unsigned step=0; step<3; ++step) {
          SCOPED_TRACE(step);
          packet.Begin(step+1);
          packet.loads = original_loads;
          const auto accepted = packet.accepted;
          RunSerialHost(packet.Input());
          ASSERT_EQ(packet.control.status, NodalStatus::Ok) << packet.control.node;
          SameDoubles(packet.accepted, accepted);
          for (double value : packet.trial) ASSERT_TRUE(std::isfinite(value));
          const auto tail = packet.TailOffset();
          for (const auto& row : packet.rows) {
            EXPECT_EQ(packet.trial[tail+row.secondary], 0);
            EXPECT_EQ(packet.trial[tail+Nodes+row.secondary], 0);
            EXPECT_EQ(packet.trial[tail+2*Nodes+row.secondary], 0);
            EXPECT_EQ(packet.trial[tail+3*Nodes+row.secondary], 0);
          }
          packet.Accept();
        }
      }
    }
  }
}

TEST(CinParallelSerialFixture, FaultPacketsExerciseExactStagePriorityBeforeGpuComparison) {
  using F = Fault;
  for (auto fault : {F::AngleBeforeInverse, F::InverseBeforeAngle, F::PendingBeforeMotion,
      F::GeometryBeforeMotion, F::ScreenBeforeMotion, F::OrdinaryBeforeGroup,
      F::GroupZeroOrientationBeforeGroupOnePrimary}) {
    SCOPED_TRACE(int(fault));
    Packet p(true, true);
    Inject(p, fault);
    const auto accepted = p.accepted;
    RunSerialHost(p.Input());
    ASSERT_NE(p.control.status, NodalStatus::Ok);
    SameDoubles(p.accepted, accepted);
    if (fault == F::AngleBeforeInverse || fault == F::OrdinaryBeforeGroup) {
      EXPECT_EQ(p.control.status, NodalStatus::StepTooLarge);
      EXPECT_EQ(p.control.node, 127u);
      EXPECT_EQ(p.control.limit.dt, 0);
    } else if (fault == F::InverseBeforeAngle) {
      EXPECT_EQ(p.control.status, NodalStatus::InvalidOutput);
      EXPECT_EQ(p.control.node, 127u);
    } else if (fault == F::PendingBeforeMotion) {
      EXPECT_EQ(p.control.status, NodalStatus::MissingStepAdmission);
      EXPECT_EQ(p.control.node, p.rows.back().secondary);
    } else if (fault == F::GeometryBeforeMotion) {
      EXPECT_EQ(p.control.status, NodalStatus::InvalidOutput);
      EXPECT_LT(p.control.node, 127u);
    } else if (fault == F::ScreenBeforeMotion) {
      EXPECT_EQ(p.control.status, NodalStatus::StepTooLarge);
      EXPECT_EQ(p.control.node, 260u);
      EXPECT_GT(p.control.limit.dt, 0);
      EXPECT_LT(p.control.limit.dt, H);
    } else {
      EXPECT_EQ(p.control.status, NodalStatus::InvalidOutput);
      EXPECT_EQ(p.control.node, p.members[0].node);
    }
  }
}
} // namespace tl::fea::cin_parallel_test
