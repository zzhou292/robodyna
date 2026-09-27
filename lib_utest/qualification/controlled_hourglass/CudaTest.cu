// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <cuda_runtime.h>
namespace controlled_test {
struct DeviceState { Case value;c::Result result;c::Status status=c::Status::InvalidInput; };
__global__ void Evaluate(DeviceState* rows,unsigned count,bool accept) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=count)return;
  auto& r=rows[i];r.status=c::EvaluateLaw42(r.value.input,r.value.state,r.result);
  if(accept&&r.status==c::Status::Success){r.value.state=r.result.proposed_state;
    r.value.input.internal_energy_density_j_m3=r.result.internal_energy_density_j_m3;}
}
class ControlledHourglassCuda:public ::testing::Test {
 protected:
  DeviceState* device=nullptr;
  void SetUp() override {int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);ASSERT_EQ(cudaSetDevice(0),cudaSuccess);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device),128*sizeof(DeviceState)),cudaSuccess);}
  void TearDown() override {if(device)EXPECT_EQ(cudaFree(device),cudaSuccess);}
  void Upload(const std::vector<DeviceState>& rows) {ASSERT_LE(rows.size(),128);ASSERT_EQ(cudaMemcpy(device,rows.data(),rows.size()*sizeof(DeviceState),cudaMemcpyHostToDevice),cudaSuccess);}
  void Run(std::vector<DeviceState>& rows,bool accept=false) {Evaluate<<<2,64>>>(device,rows.size(),accept);ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(rows.data(),device,rows.size()*sizeof(DeviceState),cudaMemcpyDeviceToHost),cudaSuccess);}
};
TEST_F(ControlledHourglassCuda, AllSlotPacketsMatchNativeAndRepeatExactly) {
  std::vector<DeviceState> rows;for(const auto& v:BasisCases())rows.push_back({v});Upload(rows);ASSERT_FALSE(HasFailure());
  Run(rows);ASSERT_FALSE(HasFailure());const auto first=rows;
  for(const auto& row:rows){ASSERT_EQ(row.status,c::Status::Success);Compare(row.result,Native(row.value));}
  Run(rows);ASSERT_FALSE(HasFailure());for(unsigned n=0;n<rows.size();++n)EXPECT_EQ(Values(rows[n].result),Values(first[n].result));
}
TEST_F(ControlledHourglassCuda, DeviceOwnedHistoryMatchesNativeFor32Intervals) {
  std::vector<DeviceState> rows(8);std::vector<Case> native;
  for(unsigned i=0;i<rows.size();++i){rows[i].value=Moving();rows[i].value.input.material_sound_speed_m_s=250+120*i;native.push_back(rows[i].value);}
  Upload(rows);ASSERT_FALSE(HasFailure());
  for(unsigned step=0;step<32;++step) {
    if(step){for(unsigned n=0;n<rows.size();++n){const double sign=step%2?-1.:1.;
      for(unsigned i=0;i<8;++i){rows[n].value.input.local_velocity_m_s[i].z=sign*(i%2?.7:-.7);native[n].input.local_velocity_m_s[i].z=rows[n].value.input.local_velocity_m_s[i].z;}}
      Upload(rows);ASSERT_FALSE(HasFailure());}
    Run(rows,true);ASSERT_FALSE(HasFailure());
    for(unsigned n=0;n<rows.size();++n){ASSERT_EQ(rows[n].status,c::Status::Success);const auto expected=Native(native[n]);Compare(rows[n].result,expected);AcceptNative(native[n],expected);}
  }
}
TEST_F(ControlledHourglassCuda, MixedFailuresKeepAcceptedStateAndAllOutputThenRepair) {
  const auto valid=Moving();const auto before=Check(valid);std::vector<DeviceState> rows(4);
  for(auto& row:rows){row.value=valid;row.result=before;}
  rows[0].value.input.poisson_ratio=std::nextafter(c::MaximumPoissonRatio,1.);
  rows[1].value.input.projection[2][1]=std::numeric_limits<double>::quiet_NaN();
  rows[2].value.state.force_n[1][3]=std::numeric_limits<double>::max();
  const auto initial=rows;Upload(rows);ASSERT_FALSE(HasFailure());Run(rows,true);ASSERT_FALSE(HasFailure());
  for(unsigned n=0;n<3;++n){EXPECT_NE(rows[n].status,c::Status::Success);EXPECT_EQ(Values(rows[n].result),Values(before));
    for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)EXPECT_EQ(rows[n].value.state.force_n[k][h],initial[n].value.state.force_n[k][h]);}
  ASSERT_EQ(rows[3].status,c::Status::Success);Compare(rows[3].result,Native(valid));
  for(auto& row:rows)row.value=valid;Upload(rows);ASSERT_FALSE(HasFailure());Run(rows,true);ASSERT_FALSE(HasFailure());
  for(const auto& row:rows){ASSERT_EQ(row.status,c::Status::Success);Compare(row.result,Native(valid));}
}
TEST_F(ControlledHourglassCuda, NativeUpperNuAndFourthModeWorkRemainDistinct) {
  std::vector<DeviceState> rows(2);rows[0].value=Base();rows[0].value.input.poisson_ratio=c::MaximumPoissonRatio;
  rows[1].value=Base();const double signs[]{1,-1,1,-1,-1,1,-1,1};
  for(unsigned n=0;n<8;++n)rows[1].value.input.local_velocity_m_s[n].y=signs[n];
  rows[1].value.input.internal_energy_density_j_m3=1e30;
  Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());
  for(const auto& row:rows){ASSERT_EQ(row.status,c::Status::Success);Compare(row.result,Native(row.value));}
  EXPECT_DOUBLE_EQ(rows[1].result.modal_velocity_m_s[1][3],1./8.);
  EXPECT_NE(rows[1].result.work_j,0);EXPECT_EQ(rows[1].result.internal_energy_density_j_m3,1e30);
}
} // namespace controlled_test
