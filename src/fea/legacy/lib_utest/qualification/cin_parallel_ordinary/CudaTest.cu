// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DevicePacket.h"
#include "Fault.h"
#include <gtest/gtest.h>

namespace tl::fea::cin_parallel_test {
namespace {
__global__ void ReduceFailure(cin_advance::FailureKey* key, cin_advance::FailureKey value) {
  atomicMin(key, value);
}
void RunPacket(Packet& p, bool parallel) {
  DevicePacket device(p);
  device.Run(parallel);
  device.Download(p);
}
}
TEST(CinParallelCuda, IntegerReductionKeepsNodePriorityForBothForcedArrivalOrders) {
  using namespace cin_advance;
  const FailureKey early = EncodeFailure(127, NodalStatus::StepTooLarge);
  const FailureKey late = EncodeFailure(256, NodalStatus::InvalidOutput);
  void* allocation = nullptr;
  ASSERT_EQ(cudaMalloc(&allocation, sizeof(FailureKey)), cudaSuccess);
  std::unique_ptr<void, DeviceDelete> owned(allocation);
  auto* raw = static_cast<FailureKey*>(allocation);
  for (bool reverse : {false, true}) {
    const FailureKey sentinel = NoFailure;
    ASSERT_EQ(cudaMemcpy(raw, &sentinel, sizeof(sentinel), cudaMemcpyHostToDevice), cudaSuccess);
    ReduceFailure<<<1,1>>>(raw, reverse?late:early);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    ReduceFailure<<<1,1>>>(raw, reverse?early:late);
    ASSERT_EQ(cudaGetLastError(), cudaSuccess);
    FailureKey observed = NoFailure;
    ASSERT_EQ(cudaMemcpy(&observed, raw, sizeof(observed), cudaMemcpyDeviceToHost), cudaSuccess);
    EXPECT_EQ(observed, early);
  }
}
TEST(CinParallelCuda, EverySuccessfulFieldMatchesFrozenSerialAcrossBlocksAndIntervals) {
  for (bool groups : {false, true}) {
    for (bool capture : {false, true}) {
      for (bool screen : {false, true}) {
        SCOPED_TRACE(groups);
        SCOPED_TRACE(capture);
        SCOPED_TRACE(screen);
        Packet serial(groups, capture);
        if (screen) serial.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
        Packet parallel = serial;
        const auto original_loads = serial.loads;
        for (unsigned step=0; step<3; ++step) {
          SCOPED_TRACE(step);
          serial.Begin(step+1);
          parallel.Begin(step+1);
          serial.loads = original_loads;
          parallel.loads = original_loads;
          const auto before = parallel.accepted;
          RunPacket(serial, false);
          RunPacket(parallel, true);
          ASSERT_EQ(serial.control.status, NodalStatus::Ok);
          ASSERT_EQ(parallel.control.status, NodalStatus::Ok);
          SameDoubles(parallel.accepted, before);
          SameSuccessfulPacket(serial, parallel);
          EXPECT_EQ(parallel.failure, cin_advance::NoFailure);
          serial.Accept();
          parallel.Accept();
        }
      }
    }
  }
}

TEST(CinParallelCuda, FailuresPreserveSerialStageOrderAndAcceptedStateThenRetryExactly) {
  using F = Fault;
  for (auto fault : {F::AngleBeforeInverse, F::InverseBeforeAngle, F::PendingBeforeMotion,
      F::GeometryBeforeMotion, F::ScreenBeforeMotion, F::OrdinaryBeforeGroup,
      F::GroupZeroOrientationBeforeGroupOnePrimary}) {
    SCOPED_TRACE(int(fault));
    Packet initial(true, true);
    Packet serial = initial;
    Inject(serial, fault);
    Packet parallel = serial;
    const auto before = parallel.accepted;
    RunPacket(serial, false);
    RunPacket(parallel, true);
    ASSERT_NE(serial.control.status, NodalStatus::Ok);
    SameControl(serial.control, parallel.control);
    SameDoubles(serial.accepted, before);
    SameDoubles(parallel.accepted, before);
    if (fault == F::PendingBeforeMotion || fault == F::GeometryBeforeMotion ||
        fault == F::ScreenBeforeMotion) {
      EXPECT_EQ(parallel.failure, initial.failure);
      SameDoubles(parallel.capture, initial.capture);
    } else if (fault != F::GroupZeroOrientationBeforeGroupOnePrimary) {
      EXPECT_EQ(parallel.failure, cin_advance::EncodeFailure(127, serial.control.status));
      SameDoubles(parallel.capture, initial.capture);
      for (std::size_t i=19*Nodes; i<parallel.TailOffset(); ++i) {
        EXPECT_EQ(parallel.trial[i], before[i]);
      }
    }
    // Failed private trial writes may differ. Discard both completely; retry
    // from the same accepted state and original loads under a fresh attempt.
    serial = initial;
    parallel = initial;
    serial.Begin(2);
    parallel.Begin(2);
    RunPacket(serial, false);
    RunPacket(parallel, true);
    SameSuccessfulPacket(serial, parallel);
  }
}
} // namespace tl::fea::cin_parallel_test
