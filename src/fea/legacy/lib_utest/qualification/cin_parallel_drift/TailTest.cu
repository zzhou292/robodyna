// SPDX-License-Identifier: AGPL-3.0-or-later
#include "DeviceFixture.cuh"
#include "lib_src/constraints/NodalRigidGroupCandidate.h"
#include "lib_src/solvers/cin_advance/Capture.h"
#include "FrozenTail.cuh"

namespace tl::fea::cin_drift_test {
namespace {
__global__ void SerialDrift(cin_advance::Input input) {
  if (input.control->status != NodalStatus::Ok) return;
  frozen::Drift(input);
}
cudaError_t Serial(const cin_advance::Input& input, cudaStream_t stream) {
  SerialDrift<<<1, 1, 0, stream>>>(input);
  const auto error = cudaGetLastError();
  return error == cudaSuccess ? cin_advance::capture::Launch(input, stream) : error;
}
cudaError_t Prepared(const cin_advance::Input& input, cudaStream_t stream) {
  return WithDrift(input, stream, [](const cin_advance::Input& source, cudaStream_t s) {
    const auto error = drift::Launch(source, s);
    return error == cudaSuccess ? cin_advance::capture::Launch(source, s) : error;
  });
}
cudaError_t SerialRecovery(const cin_advance::Input& input, cudaStream_t stream) {
  frozen_tail::CompleteCin<<<1, 1, 0, stream>>>(input, true);
  const auto error = cudaGetLastError();
  return error == cudaSuccess ? cin_advance::capture::Launch(input, stream) : error;
}
cudaError_t FailedRecovery(const cin_advance::Input& input, cudaStream_t stream) {
  return cin_recovery_test::WithRecovery(input, stream,
      [](const cin_advance::Input& source, cudaStream_t s) {
        return WithDrift(source, s, [](const cin_advance::Input& source, cudaStream_t s) {
          const auto error = recovery::Launch(source, s);
          return error == cudaSuccess ? cin_advance::capture::Launch(source, s) : error;
        }, true);
      });
}
}
TEST(CinDriftCuda, FrozenDriftKeepsFailingAxisAndQuaternionPrefixesForEveryBlockBoundary) {
  for (unsigned failed : {0, 1, 127, 128}) for (unsigned fault = 0; fault < 7; ++fault) {
    SCOPED_TRACE(failed);
    SCOPED_TRACE(fault);
    auto old = DriftPacket(129, true);
    std::reverse(old.rows.begin(), old.rows.end());
    const auto node = old.rows[failed].secondary;
    if (fault < 3) FailPosition(old, failed, fault);
    if (fault == 3) old.accepted[9*packet::Nodes+4*node] = 2;
    if (fault == 4) {
      old.accepted[9*packet::Nodes+4*node] = std::numeric_limits<double>::quiet_NaN();
      old.trial[6*packet::Nodes+3*node] = DBL_MAX;
    }
    if (fault == 5) old.durations.drift_dt = -0.0;
    if (fault == 6) old.durations.drift_dt = -packet::H;
    auto current = old;
    const auto accepted = current.accepted;
    packet::DevicePacket serial(old), parallel(current);
    serial.RunWith(Serial);
    parallel.RunWith(Prepared);
    serial.Download(old);
    parallel.Download(current);
    if (fault < 5) {
      ASSERT_NE(old.control.status, NodalStatus::Ok);
      EXPECT_EQ(old.control.node, node);
    } else ASSERT_EQ(old.control.status, NodalStatus::Ok);
    packet::SameDoubles(current.accepted, accepted);
    Same(old, current);
  }
}
TEST(CinDriftCuda, RecoveryFailurePrecedesEarlierDriftFailureWithoutTouchingDriftTailOrCapture) {
  for (unsigned fault = 0; fault < 3; ++fault) {
    auto old = cin_recovery_test::MotionPacket(129);
    const auto first = old.rows.front().secondary;
    const auto last = old.rows.back().secondary;
    old.patches.back() = {};
    if (fault == 1) old.accepted[9*packet::Nodes+4*first] = 2;
    if (fault == 2) old.control.status = NodalStatus::StaleTrial;
    auto current = old;
    packet::DevicePacket serial(old), parallel(current);
    serial.RunWith(SerialRecovery);
    parallel.RunWith(FailedRecovery);
    serial.Download(old);
    parallel.Download(current);
    ASSERT_NE(old.control.status, NodalStatus::Ok);
    if (fault < 2) EXPECT_EQ(old.control.node, last);
    Same(old, current);
  }
}
} // namespace tl::fea::cin_drift_test
