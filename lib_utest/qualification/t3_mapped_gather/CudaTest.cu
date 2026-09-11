// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "SerialAssembly.cuh"
#include <cuda_runtime.h>

namespace t3_gather_test {
struct DeviceFixture {
  Fixture source;
  b::Storage* storage=nullptr;
  Inputs* input=nullptr;
  DeviceFixture() {
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&storage),source.layout.bytes),cudaSuccess);
    EXPECT_EQ(cudaMallocManaged(reinterpret_cast<void**>(&input),sizeof(Inputs)),cudaSuccess);
  }
  ~DeviceFixture() { cudaFree(input);cudaFree(storage); }
  void Upload(unsigned epoch) {
    source.Prepare(epoch);
    Restore();
  }
  void Restore() {
    std::memcpy(storage,source.arena.data(),source.layout.bytes);
    *storage=source.layout.Rebase(*source.host,storage);
    *input=source.input;input->mixed.law=input->law;
  }
  void Run(unsigned epoch,bool serial) {
    const auto view=input->View(epoch);const auto cin=input->Cin();
    if (serial) LaunchSerialAssembly(storage,&storage->slab[0],view,cin,&input->mixed,epoch==0);
    else b::LaunchMappedAssembly(storage,&storage->slab[0],view,cin,&input->mixed,epoch==0);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
  }
};
void SameControl(const DeviceFixture& a,const DeviceFixture& b) {
  EXPECT_EQ(a.storage->control.status,b.storage->control.status);
  EXPECT_EQ(a.storage->control.element,b.storage->control.element);
  EXPECT_EQ(a.storage->control.node,b.storage->control.node);
  EXPECT_EQ(a.input->result.status,b.input->result.status);
  EXPECT_EQ(a.input->result.node,b.input->result.node);
  EXPECT_EQ(a.input->bounds.valid,b.input->bounds.valid);
}
TEST(T3MappedGatherCuda,VirginAcceptedMasksAndCancellationMatchFrozenSerialBits) {
  DeviceFixture gather,serial;
  for (unsigned epoch=0;epoch<3;++epoch) for (unsigned mask=0;mask<8;++mask) {
    SCOPED_TRACE(::testing::Message()<<"epoch="<<epoch<<" mask="<<mask);
    gather.Upload(epoch);serial.Upload(epoch);
    for (unsigned parent=0;parent<3;++parent) if (epoch && (mask&(1u<<parent))) {
      for (auto* storage:{gather.storage,serial.storage}) {
        auto& force=storage->slab[0].element[parent];
        Remove(storage->model.element[parent].reference,force);
      }
    }
    gather.Run(epoch,false);serial.Run(epoch,true);
    ASSERT_EQ(gather.storage->control.status,q::BatchStatus::Success);
    SameControl(gather,serial);Compare(*gather.input,*serial.input);
    // Assembly must never mutate a material/force cache or accepted selector.
    for (unsigned parent=0;parent<Parents;++parent) {
      const auto& a=gather.storage->slab[0].element[parent];const auto& s=serial.storage->slab[0].element[parent];
      EXPECT_EQ(std::memcmp(&a,&s,sizeof(a)),0);
    }
  }
}
TEST(T3MappedGatherCuda,FirstErrorIdentityWholeDestinationRollbackAndRetry) {
  DeviceFixture gather,serial;
  for (unsigned fault=0;fault<4;++fault) {
    gather.Upload(1);serial.Upload(1);
    for (auto* fixture:{&gather,&serial}) {
      auto& input=*fixture->input;auto* results=fixture->storage->slab[0].element;
      if (fault==0) results[3].internal_force[2].x=1; // Late skin invalidity.
      if (fault==1) input.orientation[4*3]=0; // Last physical parent's node.
      if (fault==2) { // Earlier addition failure outranks later parent validation.
        results[2].internal_force[2].x=std::numeric_limits<double>::quiet_NaN();
        results[0].internal_force[1].x=-std::numeric_limits<double>::max();
        input.values[0][0]=std::numeric_limits<double>::max();
      }
      if (fault==3) { // Same-parent node validation outranks its addition failure.
        input.orientation[4*2]=0;
        results[0].internal_force[1].x=-std::numeric_limits<double>::max();
        input.values[0][0]=std::numeric_limits<double>::max();
      }
    }
    const auto original=*gather.input;
    gather.Run(1,false);serial.Run(1,true);
    EXPECT_NE(gather.storage->control.status,q::BatchStatus::Success);
    SameControl(gather,serial);Compare(*gather.input,original);
    gather.Upload(1);serial.Upload(1);
    gather.Run(1,false);serial.Run(1,true);
    ASSERT_EQ(gather.storage->control.status,q::BatchStatus::Success);
    SameControl(gather,serial);Compare(*gather.input,*serial.input);
  }
}
TEST(T3MappedGatherCuda,RemovedSavedStateRetainsCurrentForceAndNoStiffness) {
  DeviceFixture gather,serial;
  gather.Upload(1);
  serial.Upload(1);
  for (auto* fixture:{&gather,&serial}) {
    for (unsigned parent=0;parent<3;++parent) {
      Remove(fixture->storage->model.element[parent].reference,
          fixture->storage->slab[0].element[parent],true);
    }
  }
  const auto before=*gather.input;
  gather.Run(1,false);
  serial.Run(1,true);
  ASSERT_EQ(gather.storage->control.status,q::BatchStatus::Success);
  SameControl(gather,serial);
  Compare(*gather.input,*serial.input);
  EXPECT_NE(Bits(gather.input->values[0][0]),Bits(before.values[0][0]));
  for (unsigned channel=6;channel<8;++channel) for (unsigned node=0;node<Nodes;++node) {
    EXPECT_EQ(Bits(gather.input->values[channel][node]),Bits(before.values[channel][node]));
  }
}
} // namespace t3_gather_test
