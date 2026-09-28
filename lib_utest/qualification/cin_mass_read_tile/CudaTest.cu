// SPDX-License-Identifier: AGPL-3.0-or-later
#include "lib_src/solvers/cin_advance/NumericalMassRead.cuh"
#include <gtest/gtest.h>
#include <cuda_runtime.h>
#include <vector>
#include <limits>
#include <cstring>
#include <cmath>
#include <algorithm>
namespace tl::fea::cin_mass_read_test {
namespace gather=cin_advance::force_gather;
namespace cin=constraints::tied_shell::cin;
using Row=cin::detail::PreparedForceRow;
struct Answer {double value;int success;};
// Frozen source recurrence from1c8cf7e7; independent of the candidate helpers.
bool Frozen(const std::vector<Row>& rows,double incoming,double& output) {
  double next=incoming;
  for(const auto& row:rows){
    if(!row.report)return false;
    next=next+4*row.transferred_coefficients.master[0].mass-row.secondary_mass;
    if(!std::isfinite(next))return false;
  }
  output=next;return true;
}
__global__ void Evaluate(const Row* rows,unsigned count,const double* incoming,Answer* result) {
  __shared__ gather::mass_read::Tile tile;
  cin::StageView source;source.row_count=count;
  cin::ForceTrial trial;trial.numerical_mass=const_cast<double*>(incoming);
  const bool valid=gather::mass_read::Evaluate(source,trial,rows,tile,result->value);
  if(!threadIdx.x)result->success=valid;
}
std::uint64_t Bits(double x){std::uint64_t bits;std::memcpy(&bits,&x,sizeof bits);return bits;}
void Compare(std::vector<Row> rows,double incoming,unsigned repeat=1) {
  double expected=-123.25;const auto success=Frozen(rows,incoming,expected);
  Row* device_rows=nullptr;double* device_incoming=nullptr;Answer* result=nullptr;
  ASSERT_EQ(cudaSuccess,cudaMalloc(&device_rows,std::max<std::size_t>(1,rows.size())*sizeof(Row)));
  ASSERT_EQ(cudaSuccess,cudaMalloc(&device_incoming,sizeof(double)));
  ASSERT_EQ(cudaSuccess,cudaMalloc(&result,sizeof(Answer)));
  if(!rows.empty())ASSERT_EQ(cudaSuccess,cudaMemcpy(device_rows,rows.data(),rows.size()*sizeof(Row),cudaMemcpyHostToDevice));
  ASSERT_EQ(cudaSuccess,cudaMemcpy(device_incoming,&incoming,sizeof(double),cudaMemcpyHostToDevice));
  for(unsigned n=0;n<repeat;++n){
    Answer value{-123.25,-1};ASSERT_EQ(cudaSuccess,cudaMemcpy(result,&value,sizeof value,cudaMemcpyHostToDevice));
    Evaluate<<<1,gather::mass_read::Threads>>>(device_rows,static_cast<unsigned>(rows.size()),device_incoming,result);
    ASSERT_EQ(cudaSuccess,cudaGetLastError());ASSERT_EQ(cudaSuccess,cudaDeviceSynchronize());
    ASSERT_EQ(cudaSuccess,cudaMemcpy(&value,result,sizeof value,cudaMemcpyDeviceToHost));
    EXPECT_EQ(value.success,success);EXPECT_EQ(Bits(value.value),Bits(expected));
  }
  EXPECT_EQ(cudaSuccess,cudaFree(result));EXPECT_EQ(cudaSuccess,cudaFree(device_incoming));EXPECT_EQ(cudaSuccess,cudaFree(device_rows));
}
TEST(CinMassReadTileCuda,ZeroSignsAndCancellationAcrossEveryTileBoundary) {
  for(unsigned count:{0,1,63,64,65,127,128,129,11165})for(double seed:{0.,-0.,1.}) {
    SCOPED_TRACE(count);
    SCOPED_TRACE(Bits(seed));std::vector<Row> rows(count);
    for(unsigned i=0;i<count;++i){
      auto& row=rows[i];row.transferred_coefficients.master[0].mass=i%3==0?2.5e15:.25;
      row.secondary_mass=i%3==0?1e16:0.;
    }
    Compare(std::move(rows),seed);
  }
}
TEST(CinMassReadTileCuda,FailedRowsNeverPublishAndAllLanesReachBarriers) {
  for(unsigned bad:{0,63,64,127,128}) {
    SCOPED_TRACE(bad);std::vector<Row> rows(129);
    rows[bad].report.status=cin::StageStatus::InvalidPatch;
    rows[bad].transferred_coefficients.master[0].mass=std::numeric_limits<double>::quiet_NaN();
    rows[bad].secondary_mass=std::numeric_limits<double>::infinity();
    Compare(std::move(rows),1.,8);
  }
}
TEST(CinMassReadTileCuda,OverflowPrefixBeforeLaterFailureKeepsDestinationUntouched) {
  for(unsigned bad:{0,63,64}) {
    std::vector<Row> rows(129);
    rows[bad].transferred_coefficients.master[0].mass=std::numeric_limits<double>::max()/4;
    rows[bad].secondary_mass=std::numeric_limits<double>::max();
    rows.back().report.status=cin::StageStatus::InvalidInput;
    Compare(std::move(rows),std::numeric_limits<double>::max(),8);
  }
}
} // namespace tl::fea::cin_mass_read_test
