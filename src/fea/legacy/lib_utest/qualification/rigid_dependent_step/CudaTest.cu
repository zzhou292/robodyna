// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Fixture.h"
#include <cuda_runtime.h>

namespace rigid_dependent_test {
struct Result {Trial value; r::StepStatus status; unsigned accepted=0;};
__global__ void History(Input initial,unsigned count,Result* output) {
  Result next{};
  for(unsigned step=0;step<64;++step) {
    next.status=Evaluate(initial,count,next.value);
    if(next.status!=r::StepStatus::Success)break;
    ++next.accepted;Advance(initial,next.value,count);
  }
  *output=next;
}
TEST(RigidDependentCuda, BothRigidRecurrencesKeepActualZeroCoefficientsAcross64Intervals) {
  Result* device=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(Result)),cudaSuccess);
  for(unsigned count:{2u,4u}) {
    auto in=Initial();Trial expected;
    for(unsigned step=0;step<64;++step) {
      ASSERT_EQ(Evaluate(in,count,expected),r::StepStatus::Success);Advance(in,expected,count);
    }
    History<<<1,1>>>(Initial(),count,device);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    Result out;
    ASSERT_EQ(cudaMemcpy(&out,device,sizeof(out),cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(out.status,r::StepStatus::Success);EXPECT_EQ(out.accepted,64u);
    rigid_step_test::Agreement(out.value,expected);
  }
  EXPECT_EQ(cudaFree(device),cudaSuccess);
}
} // namespace rigid_dependent_test
