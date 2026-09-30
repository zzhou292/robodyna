// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DeviceFixture.cuh"

namespace tl::fea::cin_recovery_test {
namespace {
cudaError_t Parallel(const cin_advance::Input& input, cudaStream_t stream) {
  return WithRecovery(input, stream, cin_advance::Launch);
}
void RunPair(packet::Packet& old, packet::Packet& current, bool screen) {
  packet::DevicePacket serial(old, true, screen), parallel(current, true, screen);
  serial.RunWith(LaunchFrozen);
  parallel.RunWith(Parallel);
  serial.Download(old);
  parallel.Download(current);
}
}
TEST(CinRecoveryCuda, CompleteFrozenCallerMatchesAllFieldsForThreeIntervalsAndRetry) {
  for (unsigned count : {1, 2, 127, 128, 129}) {
    for (bool groups : {false, true}) for (bool capture : {false, true}) {
      auto old = Population(count, groups, capture);
      if (capture) old.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
      auto current = old;
      const auto assembly_loads = old.loads;
      for (unsigned step = 0; step < 3; ++step) {
        SCOPED_TRACE(count);
        SCOPED_TRACE(groups);
        SCOPED_TRACE(capture);
        SCOPED_TRACE(step);
        BeginInterval(old, step+1, assembly_loads);
        BeginInterval(current, step+1, assembly_loads);
        const auto accepted = current.accepted;
        RunPair(old, current, capture);
        ASSERT_EQ(old.control.status, NodalStatus::Ok) << "node=" << old.control.node;
        packet::SameDoubles(current.accepted, accepted);
        packet::SameSuccessfulPacket(old, current);
        old.Accept();
        current.Accept();
      }
      auto retry = Population(count, groups, capture), reference = retry;
      RunPair(reference, retry, false);
      packet::SameSuccessfulPacket(reference, retry);
    }
  }
}
TEST(CinRecoveryCuda, EarlierInputOrdinaryAndRigidFailuresLeaveRecoveryAndCaptureGated) {
  for (unsigned fault = 0; fault < 4; ++fault) {
    auto old = Population(2, true, true);
    if (fault == 0) old.activity.back() = 0;
    if (fault == 1) old.loads[3*packet::Nodes+220] = 1e20;
    if (fault == 2) old.loads[3*packet::Nodes+old.members.back().node] = 1e20;
    if (fault == 3) old.control.status = NodalStatus::InvalidInput;
    auto current = old;
    RunPair(old, current, false);
    ASSERT_NE(old.control.status, NodalStatus::Ok);
    Same(old, current);
  }
}
} // namespace tl::fea::cin_recovery_test
