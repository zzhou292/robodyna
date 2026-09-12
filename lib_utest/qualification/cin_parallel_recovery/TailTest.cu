// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DeviceFixture.cuh"
#include "lib_src/constraints/NodalRigidGroupCandidate.h"
#include "lib_src/solvers/cin_advance/Capture.h"
#include "FrozenTail.cuh"

namespace tl::fea::cin_recovery_test {
namespace {
cudaError_t SerialTail(const cin_advance::Input& input, cudaStream_t stream) {
  frozen_tail::CompleteCin<<<1, 1, 0, stream>>>(input, true);
  const auto error = cudaGetLastError();
  return error == cudaSuccess ? cin_advance::capture::Launch(input, stream) : error;
}
cudaError_t ParallelTail(const cin_advance::Input& input, cudaStream_t stream) {
  return WithRecovery(input, stream, [](const cin_advance::Input& source, cudaStream_t s) {
    const auto error = recovery::Launch(source, s);
    return error == cudaSuccess ? cin_advance::capture::Launch(source, s) : error;
  });
}
}
TEST(CinRecoveryCuda, FrozenTailPreservesLateRecoveryVersusEarlierDriftFailureAndPartialFields) {
  for (unsigned fault = 0; fault < 5; ++fault) {
    auto old = MotionPacket(129);
    const auto first = old.rows.front().secondary;
    const auto last = old.rows.back().secondary;
    if (fault == 0) old.patches.back() = {};
    if (fault == 1) {
      old.patches.back() = {};
      old.accepted[9*packet::Nodes+4*first] = std::numeric_limits<double>::quiet_NaN();
    }
    if (fault == 2) {
      std::swap(old.rows.front().secondary, old.rows.back().secondary);
      old.patches.front() = {};
      old.patches.back() = {};
    }
    if (fault == 3) old.accepted[9*packet::Nodes+4*last] = std::numeric_limits<double>::quiet_NaN();
    if (fault == 4) old.durations.drift_dt = 0;
    auto current = old;
    packet::DevicePacket serial(old), parallel(current);
    serial.RunWith(SerialTail);
    parallel.RunWith(ParallelTail);
    serial.Download(old);
    parallel.Download(current);
    if (fault < 4) ASSERT_NE(old.control.status, NodalStatus::Ok);
    if (fault < 2) EXPECT_EQ(old.control.node, last);
    Same(old, current);
    auto retry = MotionPacket(129), reference = retry;
    packet::DevicePacket clean(reference), again(retry);
    clean.RunWith(SerialTail);
    again.RunWith(ParallelTail);
    clean.Download(reference);
    again.Download(retry);
    ASSERT_EQ(reference.control.status, NodalStatus::Ok);
    packet::SameSuccessfulPacket(reference, retry);
  }
}
} // namespace tl::fea::cin_recovery_test
