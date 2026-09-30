// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Cases.h"
#include "../radioss_type25_friction/CudaFixture.h"
namespace reader_solid_test {
class ReaderSolidCuda:public type25_friction_test::FrictionCuda {};
__global__ void EvaluateRows(const Case* rows,Result* out,std::size_t count) {
  for(std::size_t i=blockIdx.x*blockDim.x+threadIdx.x;i<count;i+=gridDim.x*blockDim.x) {
    Result r;r.value={11,13};
    r.status=rows[i].internal?n::EvaluateNativeInternalSolidMainCoefficient(rows[i].input,&r.value):
      n::EvaluateNativeReaderSolidMainCoefficient(rows[i].input.first,&r.value);
    out[i]=r;
  }
}
TEST_F(ReaderSolidCuda, MatchesNativeSignsAndSchedulingOrders) {
  auto cases=Cases();std::vector<Result> actual(cases.size());
  static_assert(sizeof(Case)<=RowBytes&&sizeof(Result)<=RowBytes);ASSERT_LE(cases.size(),Capacity);
  type25_friction_test::Drain drain{stream};
  for(unsigned threads:{32u,128u})for(unsigned repeat=0;repeat<4;++repeat) {
    std::reverse(cases.begin(),cases.end());
    ASSERT_EQ(cudaMemcpyAsync(input,cases.data(),cases.size()*sizeof(Case),cudaMemcpyHostToDevice,stream),cudaSuccess);
    EvaluateRows<<<2,threads,0,stream>>>(static_cast<Case*>(input),static_cast<Result*>(output),cases.size());
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(actual.data(),output,actual.size()*sizeof(Result),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    for(std::size_t i=0;i<cases.size();++i){SCOPED_TRACE(i);ASSERT_EQ(actual[i].status,n::CoefficientStatus::Ok);Same(actual[i].value,Oracle(cases[i]).value);}
  }
}
TEST_F(ReaderSolidCuda, InvalidOutputIsAtomicAndRetryMatchesNative) {
  Case c=Base();Result actual;type25_friction_test::Drain drain{stream};
  for(unsigned attempt=0;attempt<2;++attempt) {
    c.input.second_volume=attempt?12:0;
    ASSERT_EQ(cudaMemcpyAsync(input,&c,sizeof(c),cudaMemcpyHostToDevice,stream),cudaSuccess);
    EvaluateRows<<<1,1,0,stream>>>(static_cast<Case*>(input),static_cast<Result*>(output),1);
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(&actual,output,sizeof(actual),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    if(!attempt){EXPECT_EQ(actual.status,n::CoefficientStatus::InvalidInput);EXPECT_EQ(actual.value.stiffness,11);EXPECT_EQ(actual.value.characteristic_length,13);}
    else{ASSERT_EQ(actual.status,n::CoefficientStatus::Ok);Same(actual.value,Oracle(c).value);}
  }
}
}
