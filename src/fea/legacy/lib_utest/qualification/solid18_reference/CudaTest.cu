// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "SourceFixture.h"
#include <cuda_runtime.h>
#include <gtest/gtest.h>
#include <limits>
#include <vector>

namespace solid18_test {
namespace {
struct Packet {
  s::ReferenceInput input;
  s::Reference output;
  s::Status status{};
};
struct Device {
  Packet* packet = nullptr;
  ~Device() { if (packet) cudaFree(packet); }
};
__global__ void Initialize(Packet* p, unsigned count) {
  const unsigned i = blockIdx.x*blockDim.x+threadIdx.x;
  if (i < count) p[i].status = s::InitializeReference(p[i].input,p[i].output);
}
__global__ void AliasRetry(Packet* p) {
  p->status = s::InitializeReference(p->output.input(),p->output);
}
bool Send(Device& device, const Packet& p) {
  return cudaMemcpy(device.packet,&p,sizeof p,cudaMemcpyHostToDevice)==cudaSuccess;
}
bool Read(const Device& device, Packet& p) {
  return cudaMemcpy(&p,device.packet,sizeof p,cudaMemcpyDeviceToHost)==cudaSuccess;
}
}
TEST(Solid18ReferenceCuda, All908OriginalInputsAgainstIndependentNativePackets) {
  std::vector<Packet> packets(SourceCount);
  for (unsigned i = 0; i < SourceCount; ++i) packets[i].input = Source(i);
  Device device;
  const auto bytes = packets.size()*sizeof(Packet);
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.packet),bytes),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device.packet,packets.data(),bytes,cudaMemcpyHostToDevice),cudaSuccess);
  Initialize<<<(SourceCount+127)/128,128>>>(device.packet,SourceCount);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(packets.data(),device.packet,bytes,cudaMemcpyDeviceToHost),cudaSuccess);
  for (unsigned i = 0; i < SourceCount; ++i) {
    SCOPED_TRACE(packets[i].input.source_element_id);
    const auto native = Native(packets[i].input);
    ASSERT_EQ(native.status,0);
    ASSERT_EQ(packets[i].status,s::Status::Success);
    ASSERT_TRUE(Agree(Values(packets[i].output),native.values));
    ASSERT_EQ(packets[i].output.input().source_element_id,original::solids[i].id);
    for (unsigned n = 0; n < 8; ++n) {
      ASSERT_EQ(packets[i].output.source_slot(n),native.source_slot[n]);
      ASSERT_EQ(packets[i].output.input().source_node_id[n],packets[i].input.source_node_id[n]);
    }
  }
}
TEST(Solid18ReferenceCuda, LateGeometryMassFailuresAndAliasedRetryPreserveDeviceOutput) {
  Device device;
  Packet packet;
  packet.input = Distorted();
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device.packet),sizeof(Packet)),cudaSuccess);
  ASSERT_TRUE(Send(device,packet));
  Initialize<<<1,1>>>(device.packet,1);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_TRUE(Read(device,packet));
  ASSERT_EQ(packet.status,s::Status::Success);
  ASSERT_TRUE(Agree(Values(packet.output),Native(packet.input).values));
  const auto saved = Bytes(packet.output);
  const auto values = Values(packet.output);
  packet.input.position_m[6] = {.1,.1,-1};
  ASSERT_TRUE(Send(device,packet));
  Initialize<<<1,1>>>(device.packet,1);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_TRUE(Read(device,packet));
  EXPECT_EQ(packet.status,s::Status::InvalidGeometry);
  EXPECT_EQ(Bytes(packet.output),saved);
  packet.input = Distorted();
  packet.input.density_kg_m3 = std::numeric_limits<double>::max();
  ASSERT_TRUE(Send(device,packet));
  Initialize<<<1,1>>>(device.packet,1);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_TRUE(Read(device,packet));
  EXPECT_EQ(packet.status,s::Status::NonfiniteResult);
  EXPECT_EQ(Bytes(packet.output),saved);
  AliasRetry<<<1,1>>>(device.packet);
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_TRUE(Read(device,packet));
  ASSERT_EQ(packet.status,s::Status::Success);
  EXPECT_EQ(Values(packet.output),values);
  EXPECT_EQ(packet.output.input().density_kg_m3,Distorted().density_kg_m3);
}
}  // namespace solid18_test
