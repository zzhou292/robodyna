// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "Agreement.h"
#include "SourceFixture.h"
#include <cuda_runtime.h>
#include <vector>
using namespace law90_reference_test;
namespace {
struct Packet {
  s::ReferenceInput input;
  t::KinematicsInput current;
  t::ReferenceScratch reference_scratch;
  t::KinematicsScratch current_scratch;
  t::Reference reference;
  t::Kinematics accepted;
  s::Status status=s::Status::InvalidInput;
};
__global__ void ReferenceKernel(Packet* packets,unsigned count) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i>=count) return;
  auto& p=packets[i];
  p.status=t::InitializeReference90Scratch(p.input,p.reference_scratch);
  if(p.status==s::Status::Success) p.reference=p.reference_scratch.staged;
}
__global__ void CurrentKernel(Packet* packets,unsigned count) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i>=count) return;
  auto& p=packets[i];
  p.status=t::EvaluateKinematics90Scratch(p.reference,p.current,p.current_scratch);
  if(p.status==s::Status::Success) p.accepted=p.current_scratch.staged;
}
struct Device {
  Packet* value=nullptr;
  ~Device(){if(value) cudaFree(value);}
};
void Read(Device& device,std::vector<Packet>& host) {
  ASSERT_EQ(cudaGetLastError(),cudaSuccess);
  ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(host.data(),device.value,host.size()*sizeof(Packet),cudaMemcpyDeviceToHost),cudaSuccess);
}
void UploadCurrent(Device& device,const t::KinematicsInput& current) {
  ASSERT_EQ(cudaMemcpy(&device.value[0].current,&current,sizeof(current),cudaMemcpyHostToDevice),cudaSuccess);
}
}
TEST(Law90Solid18Cuda, DeviceScratchNativePathAndLateRollbackRetry) {
  int count=0;
  ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);
  ASSERT_GT(count,0);
  std::vector<Packet> host(1);
  host[0].input=Distorted();
  host[0].current=Current(host[0].input);
  Device device;
  ASSERT_EQ(cudaMalloc(&device.value,sizeof(Packet)),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device.value,host.data(),sizeof(Packet),cudaMemcpyHostToDevice),cudaSuccess);
  ReferenceKernel<<<1,1>>>(device.value,1);
  Read(device,host);
  ASSERT_FALSE(::testing::Test::HasFatalFailure());
  ASSERT_EQ(host[0].status,s::Status::Success);
  const auto native=ReferenceOracle(host[0].input);
  ASSERT_EQ(native.status,0);
  ASSERT_TRUE(ReferenceAgreement(ReferenceValues(host[0].reference),native.values));
  for(unsigned step=0;step<64;++step) {
    SCOPED_TRACE(step);
    const auto current=step ? Path(host[0].input,.5*std::sin(step*.09)) : Current(host[0].input);
    const auto expected=CurrentOracle(host[0].input,current);
    ASSERT_EQ(expected.status,0);
    UploadCurrent(device,current);
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    CurrentKernel<<<1,1>>>(device.value,1);
    Read(device,host);
    ASSERT_FALSE(::testing::Test::HasFatalFailure());
    ASSERT_EQ(host[0].status,s::Status::Success);
    ASSERT_TRUE(CurrentAgreement(CurrentValues(host[0].accepted),expected.values));
    if(step==31) {
      const auto before=Bytes(host[0].accepted);
      const auto ref_before=Bytes(host[0].reference);
      auto bad=current;
      bad.velocity_m_s[7].z=std::numeric_limits<double>::max();
      UploadCurrent(device,bad);
      ASSERT_FALSE(::testing::Test::HasFatalFailure());
      CurrentKernel<<<1,1>>>(device.value,1);
      Read(device,host);
      ASSERT_FALSE(::testing::Test::HasFatalFailure());
      ASSERT_EQ(host[0].status,s::Status::NonfiniteResult);
      ASSERT_EQ(Bytes(host[0].accepted),before);
      ASSERT_EQ(Bytes(host[0].reference),ref_before);
      UploadCurrent(device,current);
      CurrentKernel<<<1,1>>>(device.value,1);
      Read(device,host);
      ASSERT_FALSE(::testing::Test::HasFatalFailure());
      ASSERT_EQ(host[0].status,s::Status::Success);
      ASSERT_TRUE(CurrentAgreement(CurrentValues(host[0].accepted),expected.values));
    }
  }
  RecordProperty("device_packet_bytes",static_cast<int>(sizeof(Packet)));
  RecordProperty("reference_scratch_bytes",static_cast<int>(sizeof(t::ReferenceScratch)));
  RecordProperty("current_scratch_bytes",static_cast<int>(sizeof(t::KinematicsScratch)));
}
TEST(Law90Solid18Cuda, AllOriginalReferenceAndCurrentPackets) {
  std::vector<Packet> host(fixture::element_count);
  for(unsigned row=0;row<host.size();++row) {
    host[row].input=Original(row);
    host[row].current=OriginalCurrent(host[row].input);
  }
  Device device;
  const auto bytes=host.size()*sizeof(Packet);
  ASSERT_EQ(cudaMalloc(&device.value,bytes),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(device.value,host.data(),bytes,cudaMemcpyHostToDevice),cudaSuccess);
  const unsigned blocks=(host.size()+63)/64;
  ReferenceKernel<<<blocks,64>>>(device.value,host.size());
  Read(device,host);
  ASSERT_FALSE(::testing::Test::HasFatalFailure());
  for(unsigned row=0;row<host.size();++row) {
    SCOPED_TRACE(host[row].input.source_element_id);
    ASSERT_EQ(host[row].status,s::Status::Success);
    const auto expected=ReferenceOracle(host[row].input);
    ASSERT_EQ(expected.status,0);
    ASSERT_TRUE(ReferenceAgreement(ReferenceValues(host[row].reference),expected.values));
    for(unsigned n=0;n<8;++n) ASSERT_EQ(host[row].reference.source_slot(n),expected.source_slot[n]);
  }
  CurrentKernel<<<blocks,64>>>(device.value,host.size());
  Read(device,host);
  ASSERT_FALSE(::testing::Test::HasFatalFailure());
  for(unsigned row=0;row<host.size();++row) {
    SCOPED_TRACE(host[row].input.source_element_id);
    ASSERT_EQ(host[row].status,s::Status::Success);
    const auto expected=CurrentOracle(host[row].input,host[row].current);
    ASSERT_EQ(expected.status,0);
    ASSERT_TRUE(CurrentAgreement(CurrentValues(host[row].accepted),expected.values));
  }
  RecordProperty("original_cells",fixture::element_count);
  RecordProperty("explicit_device_bytes",std::to_string(bytes));
}
