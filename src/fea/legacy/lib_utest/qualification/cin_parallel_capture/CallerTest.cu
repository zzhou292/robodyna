// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/cin_force_inputs/Fixture.h"
#include "lib_utest/qualification/cin_parallel_ordinary/DevicePacket.h"
#include "lib_utest/qualification/cin_parallel_ordinary/Fault.h"
namespace tl::fea::cin_capture_test {
using namespace cin_parallel_test;
cudaError_t LaunchFrozen(const cin_advance::Input&, cudaStream_t);
TEST(CinParallelCaptureCuda, CompleteFrozenCallerMatchesHalfFullKicksRecoveryAndCapture) {
  for (bool capture : {false, true}) for (bool groups : {false, true}) {
    Packet serial(groups, capture), parallel(groups, capture);
    serial.structural = parallel.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
    for (unsigned interval = 0; interval < 3; ++interval) {
      SCOPED_TRACE(interval);
      serial.Begin(2*interval+1);
      parallel.Begin(2*interval+1);
      cin_input_test::Seed(serial);
      cin_input_test::Seed(parallel);
      DevicePacket old(serial, true, true), now(parallel, true, true);
      old.RunWith(LaunchFrozen);
      now.Run(true);
      old.Download(serial);
      now.Download(parallel);
      SameSuccessfulPacket(parallel, serial);
      serial.Accept();
      parallel.Accept();
    }
  }
}
TEST(CinParallelCaptureCuda, EarlyAndLateSuffixFailuresKeepExactPartialRigidCaptureAndRetry) {
  for (unsigned fault = 0; fault < 10; ++fault) {
    SCOPED_TRACE(fault);
    Packet serial(true, true), parallel(true, true);
    for (auto* packet : {&serial, &parallel}) {
      cin_input_test::Seed(*packet);
      if (fault < 7) Inject(*packet, static_cast<Fault>(fault));
      if (fault == 7 || fault == 8) {
        const auto secondary = fault == 7 ? packet->rows.front().secondary : packet->rows.back().secondary;
        packet->accepted[9*Nodes+4*secondary] = 0;
      }
      if (fault == 9) packet->control.status = NodalStatus::StaleTrial;
    }
    const auto accepted = parallel.accepted;
    const auto capture_before = parallel.capture;
    DevicePacket old(serial, true, true), now(parallel, true, true);
    old.RunWith(LaunchFrozen);
    now.Run(true);
    old.Download(serial);
    now.Download(parallel);
    ASSERT_NE(parallel.control.status, NodalStatus::Ok);
    SameControl(parallel.control, serial.control);
    SameDoubles(parallel.accepted, accepted);
    SameDoubles(parallel.trial, serial.trial);
    SameDoubles(parallel.loads, serial.loads);
    SameDoubles(parallel.work, serial.work);
    SameDoubles(parallel.capture, serial.capture);
    for (std::uint32_t node = 0; node < Nodes; ++node) {
      if (parallel.member_nodes[node]) continue;
      for (unsigned axis = 0; axis < 3; ++axis) {
        EXPECT_EQ(parallel.capture[3*node+axis], capture_before[3*node+axis]);
        EXPECT_EQ(parallel.capture[3*Nodes+3*node+axis], capture_before[3*Nodes+3*node+axis]);
      }
    }
    if (fault == 6 || fault == 7 || fault == 8) EXPECT_NE(parallel.capture, capture_before);
    Packet retry(true, true), reference(true, true);
    retry.Begin(2);
    reference.Begin(2);
    cin_input_test::Seed(retry);
    cin_input_test::Seed(reference);
    DevicePacket a(retry, true, true), b(reference, true, true);
    a.Run(true);
    b.RunWith(LaunchFrozen);
    a.Download(retry);
    b.Download(reference);
    SameSuccessfulPacket(retry, reference);
  }
}
} // namespace tl::fea::cin_capture_test
