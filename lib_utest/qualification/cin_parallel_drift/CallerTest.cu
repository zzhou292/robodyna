// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DeviceFixture.cuh"

namespace tl::fea::cin_drift_test {
TEST(CinDriftCuda, CompleteFrozenCallerPreservesIntervalsCaptureLimiterAndEarlierFailures) {
  for (unsigned count : {1, 2, 127, 128, 129, 200}) {
    for (bool groups : {false, true}) for (bool capture : {false, true}) {
      auto old = cin_recovery_test::Population(count, groups, capture);
      if (capture) old.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8, true};
      auto current = old;
      const auto loads = old.loads;
      for (unsigned step = 0; step < 3; ++step) {
        SCOPED_TRACE(count);
        SCOPED_TRACE(groups);
        SCOPED_TRACE(capture);
        SCOPED_TRACE(step);
        cin_recovery_test::BeginInterval(old, step+1, loads);
        cin_recovery_test::BeginInterval(current, step+1, loads);
        const auto accepted = current.accepted;
        packet::DevicePacket serial(old, true, capture), parallel(current, true, capture);
        serial.RunWith(LaunchFrozen);
        parallel.RunWith(Parallel);
        serial.Download(old);
        parallel.Download(current);
        ASSERT_EQ(old.control.status, NodalStatus::Ok) << "node=" << old.control.node;
        packet::SameDoubles(current.accepted, accepted);
        packet::SameSuccessfulPacket(old, current);
        SameWitness(old.control.structural_limiter, current.control.structural_limiter);
        if (capture) EXPECT_NE(current.control.structural_limiter.values.kind, NodalCinLimitKind::Unavailable);
        else EXPECT_EQ(current.control.structural_limiter.values.kind, NodalCinLimitKind::Unavailable);
        old.Accept();
        current.Accept();
      }
    }
  }
  for (unsigned fault = 0; fault < 4; ++fault) {
    auto old = cin_recovery_test::Population(2, true, true);
    if (fault == 0) old.activity.back() = 0;
    if (fault == 1) old.loads[3*packet::Nodes+220] = 1e20;
    if (fault == 2) old.loads[3*packet::Nodes+old.members.back().node] = 1e20;
    if (fault == 3) old.control.status = NodalStatus::InvalidInput;
    auto current = old;
    packet::DevicePacket serial(old, true), parallel(current, true);
    serial.RunWith(LaunchFrozen);
    parallel.RunWith(Parallel);
    serial.Download(old);
    parallel.Download(current);
    ASSERT_NE(old.control.status, NodalStatus::Ok);
    Same(old, current);
  }
}
} // namespace tl::fea::cin_drift_test
