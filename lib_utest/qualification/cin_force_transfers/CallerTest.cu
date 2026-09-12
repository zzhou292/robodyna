// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include "Frozen.h"
#include "../cin_parallel_ordinary/DevicePacket.h"
#include <cuda_runtime.h>
#include <cfloat>
#include <cstring>

namespace tl::fea::cin_transfer_test {
namespace {
// Test-only guarded tail; the actual owner test exercises startup allocation.
cudaError_t WithPrepared(const cin_advance::Input& original, cudaStream_t stream) {
  transfer::Row* storage = nullptr;
  const auto count = original.model.row_count;
  auto error = cudaMalloc(&storage, (count+2)*sizeof(transfer::Row));
  if (error != cudaSuccess) return error;
  const auto finish = [&](cudaError_t status) {
    const auto released = cudaFree(storage);
    return status == cudaSuccess ? released : status;
  };
  error = cudaMemsetAsync(storage, 0xa5, (count+2)*sizeof(transfer::Row), stream);
  if (error != cudaSuccess) return finish(error);
  auto input = original;
  input.prepared_transfers = storage+1;
  error = cin_advance::Launch(input, stream);
  if (error != cudaSuccess) return finish(error);
  error = cudaStreamSynchronize(stream);
  if (error != cudaSuccess) return finish(error);
  std::vector<unsigned char> bytes((count+2)*sizeof(transfer::Row));
  error = cudaMemcpy(bytes.data(), storage, bytes.size(), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  for (std::size_t index = 0; index < sizeof(transfer::Row); ++index) {
    EXPECT_EQ(bytes[index], 0xa5);
    EXPECT_EQ(bytes[(count+1)*sizeof(transfer::Row)+index], 0xa5);
  }
  nodal_detail::Control control;
  error = cudaMemcpy(&control, input.control, sizeof(control), cudaMemcpyDeviceToHost);
  if (error != cudaSuccess) return finish(error);
  if (control.status == NodalStatus::Ok) {
    for (unsigned row = 0; row < count; ++row) {
      transfer::Row value;
      std::memcpy(&value, bytes.data()+(row+1)*sizeof(value), sizeof(value));
      EXPECT_TRUE(value.report);
      EXPECT_TRUE(value.patch.prepared());
    }
  }
  return finish(cudaSuccess);
}
void RunPair(packet::Packet& serial, packet::Packet& prepared, bool screen) {
  packet::DevicePacket old(serial, true, screen), now(prepared, true, screen);
  old.RunWith(LaunchFrozen);
  now.RunWith(WithPrepared);
  old.Download(serial);
  now.Download(prepared);
}
} // namespace

TEST(CinPreparedForceRowsCuda, FrozenFullCallerMatchesThreeIntervalsAcrossRowBlocks) {
  for (unsigned count : {1,2,127,128,129}) {
    for (bool groups : {false,true}) for (bool capture : {false,true}) {
      SCOPED_TRACE(count);
      SCOPED_TRACE(groups);
      SCOPED_TRACE(capture);
      const bool screen = capture;
      auto serial = Population(count, groups, capture);
      if (screen) serial.structural = {NodalCinStructuralProfile::NativeOrdinaryRigidTrace, .8};
      auto prepared = serial;
      for (unsigned step = 0; step < 3; ++step) {
        serial.Begin(step+1);
        prepared.Begin(step+1);
        cin_input_test::Seed(serial);
        cin_input_test::Seed(prepared);
        const auto accepted = prepared.accepted;
        RunPair(serial, prepared, screen);
        ASSERT_EQ(serial.control.status, NodalStatus::Ok) << serial.control.node;
        packet::SameSuccessfulPacket(serial, prepared);
        SamePatches(serial, prepared);
        packet::SameDoubles(prepared.accepted, accepted);
        serial.Accept();
        prepared.Accept();
      }
    }
  }
}

TEST(CinPreparedForceRowsCuda, OrderedApplyFailurePrecedesLateLeafAndKeepsExactPartialPacketThenRetry) {
  for (unsigned fault = 0; fault < 3; ++fault) {
    SCOPED_TRACE(fault);
    auto serial = Population(129, true, true);
    CenterFirstSecondary(serial);
    LatePatch(serial);
    const auto secondary = serial.rows[0].secondary;
    if (fault == 0) {
      serial.loads[serial.rows[0].masters[0]] = DBL_MAX;
      serial.loads[secondary] = DBL_MAX/8;
      for (unsigned axis = 1; axis < 6; ++axis) serial.loads[axis*packet::Nodes+secondary] = 0;
    } else if (fault == 1) {
      serial.trial.back() = DBL_MAX;
      serial.trial[serial.TailOffset()+secondary] = DBL_MAX/2;
    }
    auto prepared = serial;
    const auto accepted = prepared.accepted;
    RunPair(serial, prepared, true);
    ASSERT_EQ(serial.control.status, NodalStatus::InvalidOutput);
    packet::SameControl(serial.control, prepared.control);
    SameForce(serial, prepared);
    EXPECT_EQ(serial.failure, prepared.failure);
    packet::SameDoubles(prepared.accepted, accepted);
    auto retry = Population(129, true, true);
    retry.Begin(2);
    cin_input_test::Seed(retry);
    auto reference = retry;
    RunPair(reference, retry, true);
    ASSERT_EQ(retry.control.status, NodalStatus::Ok);
    packet::SameSuccessfulPacket(reference, retry);
    SamePatches(reference, retry);
  }
}

TEST(CinPreparedForceRowsCuda, RejectedInputsDoNotReadLeafTailOrChangeEntryInertia) {
  for (auto fault : {cin_input_test::Fault::NodeBeforeWitness, cin_input_test::Fault::NumericalBeforeWitness,
                    cin_input_test::Fault::LateRow, cin_input_test::Fault::NoRows}) {
    auto serial = Population(2, true, true);
    auto prepared = serial;
    cin_input_test::Inject(serial, fault);
    cin_input_test::Inject(prepared, fault);
    const auto before = prepared;
    RunPair(serial, prepared, false);
    ASSERT_NE(serial.control.status, NodalStatus::Ok);
    packet::SameControl(serial.control, prepared.control);
    SameForce(serial, prepared);
    packet::SameDoubles(prepared.work, before.work);
    packet::SameDoubles(prepared.accepted, before.accepted);
  }
}
} // namespace tl::fea::cin_transfer_test
