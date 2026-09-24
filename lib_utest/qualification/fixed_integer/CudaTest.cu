// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Oracle.h"
#include <cuda_runtime.h>
#include <algorithm>
namespace fixed_integer_test {
namespace {
__global__ void EvaluateRows(const Case* cases,Result* results,std::size_t count) {
  const auto row=std::size_t(blockIdx.x)*blockDim.x+threadIdx.x;
  if(row<count)results[row]=Evaluate(cases[row]);
}
// This outlives every queued use of the caller's input and local readback
// vectors, including early ASSERT returns after a later CUDA call fails.
struct Drain {
  cudaStream_t stream;
  ~Drain() { EXPECT_EQ(cudaStreamSynchronize(stream),cudaSuccess); }
};
class FixedIntegerCuda : public ::testing::Test {
 protected:
  cudaStream_t stream=nullptr;
  Case* input=nullptr;
  Result* output=nullptr;
  void SetUp() override {
    ASSERT_EQ(cudaStreamCreateWithFlags(&stream,cudaStreamNonBlocking),cudaSuccess);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&input),MaximumCases*sizeof(Case)),cudaSuccess);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&output),MaximumCases*sizeof(Result)),cudaSuccess);
  }
  void TearDown() override {
    if(stream)EXPECT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    if(output)EXPECT_EQ(cudaFree(output),cudaSuccess);
    if(input)EXPECT_EQ(cudaFree(input),cudaSuccess);
    if(stream)EXPECT_EQ(cudaStreamDestroy(stream),cudaSuccess);
  }
  void Check(const std::vector<Case>& rows,unsigned threads) {
    ASSERT_LE(rows.size(),MaximumCases);ASSERT_FALSE(rows.empty());
    std::vector<Result> actual(rows.size());
    Drain drain{stream};
    ASSERT_EQ(cudaMemcpyAsync(input,rows.data(),rows.size()*sizeof(Case),cudaMemcpyHostToDevice,stream),cudaSuccess);
    EvaluateRows<<<(rows.size()+threads-1)/threads,threads,0,stream>>>(input,output,rows.size());
    ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpyAsync(actual.data(),output,actual.size()*sizeof(Result),cudaMemcpyDeviceToHost,stream),cudaSuccess);
    ASSERT_EQ(cudaStreamSynchronize(stream),cudaSuccess);
    for(std::size_t i=0;i<rows.size();++i) {
      SCOPED_TRACE(i);Same(actual[i],Evaluate(rows[i]));CheckOracle(rows[i],actual[i]);
    }
  }
};
TEST_F(FixedIntegerCuda, CompleteCorpusMatchesHostAndUnboundedOracle) {
  Check(Corpus(),64);
}
TEST_F(FixedIntegerCuda, LaunchShapeOrderAndRepeatedPublicationPreserveAllFields) {
  auto rows=Corpus();
  for(unsigned threads:{32u,128u}) {
    for(unsigned repeat=0;repeat<4;++repeat) {
      std::reverse(rows.begin(),rows.end());Check(rows,threads);
    }
  }
}
}  // namespace
}  // namespace fixed_integer_test
