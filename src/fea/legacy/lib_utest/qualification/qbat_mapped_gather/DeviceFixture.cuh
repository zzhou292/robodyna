// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include "SerialAssembly.cuh"
#include "SerialCandidate.cuh"
#include <cuda_runtime.h>
namespace qbat_gather_test {
struct DeviceFixture {
  Fixture source;
  b::Storage* storage=nullptr;
  Inputs* input=nullptr;
  DeviceFixture() {
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&storage),source.layout.bytes),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&input),sizeof(Inputs)),cudaSuccess);
  }
  ~DeviceFixture() {cudaFree(input);cudaFree(storage);}
  void Upload(unsigned epoch,bool loads=true) {source.Prepare(epoch,loads);Restore();}
  void Restore() {
    std::memcpy(storage,source.arena.data(),source.layout.bytes);
    *storage=source.layout.Rebase(*source.host,storage);*input=source.input;
  }
  void Assemble(unsigned epoch,bool reference) {
    const auto view=input->View(epoch);const auto cin=input->Cin();
    if(reference) serial::LaunchMappedAssembly(storage,&storage->slab[0],view,cin,epoch==0);
    else b::LaunchMappedAssembly(storage,&storage->slab[0],view,cin,epoch==0);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  }
  void Measure(unsigned epoch,bool reference) {
    const auto view=input->Prepared(epoch);const auto identity=Identity(epoch);
    if(reference) serial_candidate::FinalizeCandidate<<<1,1>>>(storage,&storage->slab[0],&storage->slab[1],view,identity);
    else b::LaunchMappedMeasurements(storage,&storage->slab[0],&storage->slab[1],view,identity,Nodes);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  }
  void Candidate(unsigned epoch,bool reference) {
    const auto view=input->Prepared(epoch);const auto identity=Identity(epoch);
    if(reference) serial_candidate::LaunchCandidate(storage,&storage->slab[0],&storage->slab[1],view,identity,Parents);
    else b::LaunchCandidate(storage,&storage->slab[0],&storage->slab[1],view,identity,Parents,Nodes);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  }
};
inline void SameControl(const DeviceFixture& a,const DeviceFixture& b) {
  EXPECT_EQ(a.storage->control.status,b.storage->control.status);
  EXPECT_EQ(a.storage->control.element,b.storage->control.element);
  EXPECT_EQ(a.storage->control.node,b.storage->control.node);
  EXPECT_EQ(a.storage->control.element_status,b.storage->control.element_status);
  EXPECT_EQ(a.input->result.status,b.input->result.status);
  EXPECT_EQ(a.input->result.node,b.input->result.node);
  EXPECT_EQ(a.input->bounds.valid,b.input->bounds.valid);
}
} // namespace qbat_gather_test
