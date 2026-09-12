// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "Frozen.h"
#include "../cin_parallel_ordinary/DevicePacket.h"
#include "../cin_parallel_ordinary/Fault.h"
#include "../cin_force_inputs/Fixture.h"
#include <cstring>

namespace tl::fea::cin_group_test {
namespace {
// Test-only scratch adapter for the reused private packet. Real owner allocation
// and retry are exercised separately; no allocation occurs in production Launch.
cudaError_t WithReports(const cin_advance::Input& original, cudaStream_t stream) {
  using Report = cin_advance::groups::Report;
  Report* storage = nullptr;
  const auto count = original.groups.group_count;
  auto error = cudaMalloc(&storage, (count+2)*sizeof(Report));
  if (error != cudaSuccess) return error;
  const auto finish = [&](cudaError_t status) {
    const auto release = cudaFree(storage);
    return status == cudaSuccess ? release : status;
  };
  error = cudaMemsetAsync(storage, 0xa5, (count+2)*sizeof(Report), stream);
  if (error != cudaSuccess) return finish(error);
  auto input = original;
  input.group_reports = storage+1;
  error = cin_advance::Launch(input, stream);
  if (error != cudaSuccess) return finish(error);
  error = cudaStreamSynchronize(stream);
  if (error != cudaSuccess) return finish(error);
  std::vector<unsigned char> bytes((count+2)*sizeof(Report));
  error = cudaMemcpy(bytes.data(), storage, bytes.size(), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  for (unsigned a = 0; a < sizeof(Report); ++a) {
    EXPECT_EQ(bytes[a], 0xa5);
    EXPECT_EQ(bytes[(count+1)*sizeof(Report)+a], 0xa5);
  }
  nodal_detail::Control control;
  error = cudaMemcpy(&control, input.control, sizeof(control), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  if (control.status == NodalStatus::Ok) {
    for (unsigned g = 0; g < count; ++g) {
      Report report;
      std::memcpy(&report, bytes.data()+(g+1)*sizeof(Report), sizeof(report));
      EXPECT_EQ(report.status, NodalStatus::Ok);
      EXPECT_EQ(report.last_node, UINT32_MAX);
      EXPECT_EQ(report.minimum_dt, 0); // Motion fully overwrites screen reports.
      EXPECT_FALSE(report.visited);
      EXPECT_FALSE(report.bounded);
    }
  }
  return finish(cudaSuccess);
}
void RunPair(packet::Packet& serial, packet::Packet& parallel, bool screen) {
  packet::DevicePacket old(serial, true, screen), now(parallel, true, screen);
  old.RunWith(LaunchFrozen);
  now.RunWith(WithReports);
  old.Download(serial);
  now.Download(parallel);
}
} // namespace
TEST(CinParallelGroupsCuda, FrozenFullCallerMatchesThreeIntervalsAcrossGroupBlocks) {
  for (bool capture : {false, true}) for (bool screen : {false, true}) {
    for (unsigned count : {0, 2, 65, 129}) {
      SCOPED_TRACE(capture);
      SCOPED_TRACE(screen);
      SCOPED_TRACE(count);
      auto serial = MakePacket(count, capture);
      auto parallel = serial;
      if (screen) serial.structural = parallel.structural =
          {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
      for (unsigned step = 0; step < 3; ++step) {
        SCOPED_TRACE(step);
        serial.Begin(step+1); parallel.Begin(step+1);
        cin_input_test::Seed(serial); cin_input_test::Seed(parallel);
        const auto accepted = parallel.accepted;
        RunPair(serial, parallel, screen);
        ASSERT_EQ(serial.control.status, NodalStatus::Ok) << serial.control.node;
        packet::SameSuccessfulPacket(serial, parallel);
        packet::SameDoubles(parallel.accepted, accepted);
        serial.Accept(); parallel.Accept();
      }
    }
  }
}
TEST(CinParallelGroupsCuda, GroupOrdinalAndWithinGroupPhaseWinThenCompleteRetryMatches) {
  for (unsigned fault = 0; fault < 10; ++fault) {
    SCOPED_TRACE(fault);
    packet::Packet serial(true, true);
    ReverseGroups(serial);
    cin_input_test::Seed(serial);
    const auto first = serial.members[serial.groups[0].offset].node;
    if (fault < 4) {
      serial.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
      if (fault == 0) serial.groups[0].count = 1;
      if (fault == 1) serial.groups[1].count = 1;
      if (fault == 2) {
        serial.groups[0].mass = 0; serial.groups[1].mass = 0;
      }
      if (fault == 3) serial.work[first] = 1e30;
    } else if (fault == 4) {
      serial.groups[0].mass = 0; serial.groups[1].mass = 0;
    } else if (fault == 5) {
      serial.accepted[9*packet::Nodes+4*first] = 0;
      serial.groups[1].mass = 0;
    } else if (fault == 6) {
      serial.loads[3*packet::Nodes+first] = 1e12;
      serial.groups[1].mass = 0;
    } else if (fault == 7) {
      packet::Inject(serial, packet::Fault::OrdinaryBeforeGroup);
    } else if (fault == 8) {
      packet::Inject(serial, packet::Fault::PendingBeforeMotion);
    } else {
      serial.groups[1].mass = 0;
    }
    auto parallel = serial;
    const auto accepted = parallel.accepted;
    const auto before_capture = parallel.capture;
    const auto before_key = parallel.failure;
    RunPair(serial, parallel, fault < 4);
    ASSERT_NE(serial.control.status, NodalStatus::Ok);
    packet::SameControl(serial.control, parallel.control);
    packet::SameDoubles(parallel.accepted, accepted);
    packet::SameDoubles(serial.loads, parallel.loads);
    packet::SameDoubles(serial.work, parallel.work);
    EXPECT_EQ(serial.failure, parallel.failure);
    if (fault < 4) {
      packet::SameDoubles(parallel.trial, serial.trial);
      packet::SameDoubles(parallel.capture, before_capture);
      EXPECT_EQ(parallel.failure, before_key);
    }
    if (fault == 0) EXPECT_EQ(parallel.control.node, packet::Nodes-1);
    if (fault >= 2 && fault <= 6) EXPECT_EQ(parallel.control.node, first);
    // On group failure later groups may write private trial/capture fields.
    // Compare the published failure and accepted values, then discard/retry.
    packet::Packet retry(true, true);
    ReverseGroups(retry);
    retry.Begin(2);
    cin_input_test::Seed(retry);
    auto reference = retry;
    RunPair(reference, retry, false);
    ASSERT_EQ(retry.control.status, NodalStatus::Ok);
    packet::SameSuccessfulPacket(reference, retry);
  }
}
} // namespace tl::fea::cin_group_test
