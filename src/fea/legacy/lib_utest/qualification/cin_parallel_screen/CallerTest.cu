// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_utest/qualification/cin_force_inputs/Fixture.h"
#include "lib_utest/qualification/cin_parallel_ordinary/DevicePacket.h"
namespace tl::fea::cin_screen_test {
using namespace cin_parallel_test;
cudaError_t LaunchFrozen(const cin_advance::Input&, cudaStream_t);
TEST(CinParallelScreenCuda, CompleteFrozenCallerMatchesHalfFullKicksCaptureAndRigidBodies) {
  for (bool capture : {false, true}) for (bool groups : {false, true}) {
    Packet serial(groups, capture), parallel(groups, capture);
    serial.structural = parallel.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
    for (unsigned interval = 0; interval < 3; ++interval) {
      SCOPED_TRACE(interval);
      serial.Begin(2*interval+1);
      parallel.Begin(2*interval+1);
      cin_input_test::Seed(serial);
      cin_input_test::Seed(parallel);
      DevicePacket old(serial, true), now(parallel, true, true);
      old.RunWith(LaunchFrozen);
      now.Run(true);
      const auto observed = now.ScreenSummary();
      EXPECT_EQ(observed.invalid_node, UINT32_MAX);
      EXPECT_GT(observed.minimum_dt, 0);
      EXPECT_NE(observed.minimum_dt, -7);
      old.Download(serial);
      now.Download(parallel);
      SameSuccessfulPacket(parallel, serial);
      EXPECT_EQ(parallel.failure, cin_advance::NoFailure);
      serial.Accept();
      parallel.Accept();
    }
  }
}
TEST(CinParallelScreenCuda, ScreenErrorsPreserveMotionCaptureAndKeyThenRetryWholeCaller) {
  for (unsigned fault = 0; fault < 6; ++fault) {
    SCOPED_TRACE(fault);
    Packet serial(true, true), parallel(true, true);
    for (auto* p : {&serial, &parallel}) {
      p->structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
      cin_input_test::Seed(*p);
      if (fault == 0) p->trial[p->TailOffset()+Nodes+128] = 0; // Free rotation.
      if (fault == 1) p->structural.factor = 0;
      if (fault == 2) p->groups[0].count = 1;
      if (fault == 3) p->groups[1].offset = 99;
      if (fault == 4) p->groups[1].principal_inertia.z = 0;
      if (fault == 5) p->work[128] = 1e20; // Valid finite but rejected dt.
    }
    const auto accepted = parallel.accepted;
    const auto trial_before = parallel.trial;
    const auto capture_before = parallel.capture;
    const auto failure_before = parallel.failure;
    DevicePacket old(serial, true), now(parallel, true, true);
    old.RunWith(LaunchFrozen);
    now.Run(true);
    old.Download(serial);
    now.Download(parallel);
    ASSERT_NE(parallel.control.status, NodalStatus::Ok);
    SameControl(parallel.control, serial.control);
    cin_input_test::SameForce(parallel, serial);
    SameDoubles(parallel.accepted, accepted);
    SameDoubles(parallel.capture, capture_before);
    for (unsigned i = 0; i < 19*Nodes; ++i) EXPECT_EQ(parallel.trial[i], trial_before[i]);
    EXPECT_EQ(parallel.failure, failure_before);
    if (fault == 2) EXPECT_EQ(parallel.control.node, Nodes-1);
    // Restore valid complete source and retry from the accepted state.
    Packet retry(true, true), reference(true, true);
    retry.structural = reference.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
    retry.Begin(2);
    reference.Begin(2);
    cin_input_test::Seed(retry);
    cin_input_test::Seed(reference);
    DevicePacket a(retry, true, true), b(reference, true);
    a.Run(true);
    b.RunWith(LaunchFrozen);
    a.Download(retry);
    b.Download(reference);
    SameSuccessfulPacket(retry, reference);
  }
}
} // namespace tl::fea::cin_screen_test
