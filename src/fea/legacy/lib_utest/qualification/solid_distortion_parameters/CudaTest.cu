// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <cuda_runtime.h>
#include <type_traits>
namespace distortion_test {
struct Packet {Case value;d::Parameters result;d::Status status=d::Status::InvalidInput;};
static_assert(std::is_trivially_copyable_v<Packet>);
__global__ void Prepare(Packet* rows,unsigned count) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i<count)rows[i].status=d::PrepareParameters(rows[i].value.material,rows[i].value.input,rows[i].result);
}
class DistortionParametersCuda:public ::testing::Test {
 protected:
  Packet* device=nullptr;
  void SetUp() override {
    int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);
    ASSERT_EQ(cudaSetDevice(0),cudaSuccess);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device),128*sizeof(Packet)),cudaSuccess);
  }
  void TearDown() override {if(device)EXPECT_EQ(cudaFree(device),cudaSuccess);}
  void Upload(const std::vector<Packet>& rows) {
    ASSERT_LE(rows.size(),128);
    ASSERT_EQ(cudaMemcpy(device,rows.data(),rows.size()*sizeof(Packet),cudaMemcpyHostToDevice),cudaSuccess);
  }
  void Run(std::vector<Packet>& rows) {
    Prepare<<<2,64>>>(device,rows.size());ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(rows.data(),device,rows.size()*sizeof(Packet),cudaMemcpyDeviceToHost),cudaSuccess);
  }
};
TEST_F(DistortionParametersCuda, NativeCasesMatchAndRepeatExactly) {
  std::vector<Packet> rows;for(const auto& value:Cases())rows.push_back({value,Sentinel()});
  Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());const auto first=rows;
  for(const auto& row:rows){ASSERT_EQ(row.status,d::Status::Success);Compare(row.result,Native(row.value));}
  Run(rows);ASSERT_FALSE(HasFailure());
  for(unsigned n=0;n<rows.size();++n)Same(rows[n].result,first[n].result);
}
TEST_F(DistortionParametersCuda, SixComponentThresholdsAndHydrostaticStressMatchNative) {
  std::vector<Packet> rows;
  for(unsigned component=0;component<6;++component)for(unsigned side=0;side<2;++side) {
    auto value=Base();const double threshold=-NativeC1(value);
    value.input.cauchy_stress_pa[component]=side?std::nextafter(threshold,-std::numeric_limits<double>::infinity()):threshold;
    rows.push_back({value,Sentinel()});
  }
  auto hydro=Base();for(unsigned i=0;i<3;++i)hydro.input.cauchy_stress_pa[i]=NativeC1(hydro);
  rows.push_back({hydro,Sentinel()});Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());
  for(const auto& row:rows){ASSERT_EQ(row.status,d::Status::Success);Compare(row.result,Native(row.value));}
}
TEST_F(DistortionParametersCuda, MixedFailuresPreserveOutputAndRepair) {
  std::vector<Packet> rows(5);for(auto& row:rows)row={Base(),Sentinel()};
  rows[0].value.material.bulk_pa=0;
  rows[1].value.input.offg=0;
  rows[2].value.input.current_volume_m3=0;
  ASSERT_EQ(m::Prepare(1e307,0,1100,1e12,rows[3].value.material),m::Status::Ok);
  rows[3].value.input.current_volume_m3=1e300;
  Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());
  const d::Status statuses[]{d::Status::InvalidMaterial,d::Status::UnsupportedProfile,
    d::Status::InvalidInput,d::Status::NonfiniteResult};
  for(unsigned n=0;n<4;++n){EXPECT_EQ(rows[n].status,statuses[n]);Same(rows[n].result,Sentinel());}
  ASSERT_EQ(rows[4].status,d::Status::Success);Compare(rows[4].result,Native(rows[4].value));
  for(auto& row:rows)row.value=Base();Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());
  for(const auto& row:rows){ASSERT_EQ(row.status,d::Status::Success);Compare(row.result,Native(row.value));}
}
} // namespace distortion_test
