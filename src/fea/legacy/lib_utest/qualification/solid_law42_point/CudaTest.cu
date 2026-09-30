// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <cuda_runtime.h>
namespace law42_test {
__global__ void Compute(law::Parameters p,law::Input input,law::Result* result,law::Status* status) {
  *status=law::Update(p,input,*result);
}
TEST(Law42Cuda,NativeMaterialPointAndRejectedUpdateRetry) {
  law::Result* device=nullptr;law::Status* status=nullptr;
  ASSERT_EQ(cudaMalloc(&device,sizeof(law::Result)),cudaSuccess);
  ASSERT_EQ(cudaMalloc(&status,sizeof(law::Status)),cudaSuccess);
  law::Result prior{},actual{};law::Status code;
  ASSERT_EQ(cudaMemcpy(device,&prior,sizeof prior,cudaMemcpyHostToDevice),cudaSuccess);
  auto p=Material();auto x=Rotate(Stretch(1.3,.8,1.1),.37);
  for(unsigned step=0;step<3;++step) {
    auto attempt=x;if(step==1)attempt.total_strain[0]=-10;
    Compute<<<1,1>>>(p,attempt,device,status);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaDeviceSynchronize(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&actual,device,sizeof actual,cudaMemcpyDeviceToHost),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(&code,status,sizeof code,cudaMemcpyDeviceToHost),cudaSuccess);
    if(step==1) {
      EXPECT_EQ(code,law::Status::InvalidStretch);
      EXPECT_EQ(std::memcmp(&actual,&prior,sizeof actual),0);
    } else {
      ASSERT_EQ(code,law::Status::Ok);
      double values[13],native[13];Pack(p,actual,values);Native(p,x,native);Compare(values,native);
      prior=actual;
    }
  }
  EXPECT_EQ(cudaFree(status),cudaSuccess);EXPECT_EQ(cudaFree(device),cudaSuccess);
}
}
