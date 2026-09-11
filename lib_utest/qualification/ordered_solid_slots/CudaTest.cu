// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <cuda_runtime.h>
using namespace slot_test;
namespace {
__global__ void Scatter(Packet* p) { Execute(*p); }
struct Device {
  Packet* p=nullptr;
  ~Device(){if(p)cudaFree(p);}
  void Run(Packet& host) {
    ASSERT_EQ(cudaMemcpy(p,&host,sizeof(host),cudaMemcpyHostToDevice),cudaSuccess);
    Scatter<<<1,1>>>(p);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&host,p,sizeof(host),cudaMemcpyDeviceToHost),cudaSuccess);
  }
};
}
TEST(OrderedSolidSlotsCuda, SlotOrderAndEveryLateFailureOnDevice) {
  Device d;ASSERT_EQ(cudaMalloc(&d.p,sizeof(Packet)),cudaSuccess);
  const auto run=[&](Packet& p){ASSERT_NO_FATAL_FAILURE(d.Run(p));};
  OrderAndUntouchedRotation(run);StiffnessOrderAndRollback(run);ForceFailureAndRetry(run);
}
