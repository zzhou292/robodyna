// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <cuda_runtime.h>
namespace h24_test {
struct Row {Case input;Trial output;s::ForceStatus status=s::ForceStatus::InvalidInput;};
__global__ void Compute(Row* rows,unsigned count,bool accept) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;if(i>=count)return;
  auto& r=rows[i];r.status=Evaluate(r.input,r.output);
  if(accept&&r.status==s::ForceStatus::Success)r.input.accepted=r.output.stage.proposed_values;
}
class H24ControlledCuda:public ::testing::Test {
 protected:
  Row* device=nullptr;
  void SetUp()override {int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);ASSERT_EQ(cudaSetDevice(0),cudaSuccess);ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device),64*sizeof(Row)),cudaSuccess);}
  void TearDown()override {if(device)EXPECT_EQ(cudaFree(device),cudaSuccess);}
  void Upload(const std::vector<Row>& r){ASSERT_LE(r.size(),64);ASSERT_EQ(cudaMemcpy(device,r.data(),r.size()*sizeof(Row),cudaMemcpyHostToDevice),cudaSuccess);}
  void Run(std::vector<Row>& r,bool accept=false){Compute<<<1,64>>>(device,r.size(),accept);ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaMemcpy(r.data(),device,r.size()*sizeof(Row),cudaMemcpyDeviceToHost),cudaSuccess);}
};
TEST_F(H24ControlledCuda, NativeIntermediateStagesAndDeterministicRepeat) {
  std::vector<Row> rows;for(const auto& x:Cases())rows.push_back({x});Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());const auto first=rows;
  for(const auto& row:rows){ASSERT_EQ(row.status,s::ForceStatus::Success);Compare(row.input,row.output,Native(NativeHistory(row.input),row.input));}
  Run(rows);ASSERT_FALSE(HasFailure());for(unsigned n=0;n<rows.size();++n)EXPECT_EQ(Values(rows[n].output),Values(first[n].output));
}
TEST_F(H24ControlledCuda, DeviceCarriedStateMatches32NativeIntervals) {
  std::vector<Row> rows;for(unsigned n=0;n<4;++n){auto x=Base();x.accepted.material.internal_energy_density_j_m3=3+n;rows.push_back({x});}
  std::vector<native::History> histories;for(const auto& r:rows)histories.push_back(NativeHistory(r.input));
  for(unsigned step=0;step<32;++step){SCOPED_TRACE(step);
    for(unsigned n=0;n<rows.size();++n)Move(rows[n].input,(step%8<4?1.:-1.)*(.1+.01*step+.02*n));
    Upload(rows);ASSERT_FALSE(HasFailure());Run(rows,true);ASSERT_FALSE(HasFailure());
    for(unsigned n=0;n<rows.size();++n){SCOPED_TRACE(n);ASSERT_EQ(rows[n].status,s::ForceStatus::Success);const auto expected=Native(histories[n],rows[n].input);Compare(rows[n].input,rows[n].output,expected);
      std::copy_n(expected.full.values.begin(),22,histories[n].values.begin());rows[n].input.interval.base_time_s+=rows[n].input.interval.dt_s;++rows[n].input.interval.sample_index;}
    ASSERT_FALSE(HasFailure());
  }
}
TEST_F(H24ControlledCuda, MixedRejectionAndRepairPreserveAcceptedStateAndOutput) {
  auto x=Base();Move(x);const auto before=Check(x);std::vector<Row> rows(3);for(auto& r:rows){r.input=x;r.output=before;}
  rows[0].input.accepted.controlled_hourglass.force_n[2][3]=std::numeric_limits<double>::max();
  rows[1].input.interval.velocity_m_s[0].x=std::numeric_limits<double>::infinity();const auto initial=rows;
  Upload(rows);ASSERT_FALSE(HasFailure());Run(rows,true);ASSERT_FALSE(HasFailure());
  for(unsigned n=0;n<2;++n){EXPECT_NE(rows[n].status,s::ForceStatus::Success);EXPECT_EQ(Values(rows[n].output),Values(before));
    for(unsigned k=0;k<3;++k)for(unsigned h=0;h<4;++h)EXPECT_EQ(rows[n].input.accepted.controlled_hourglass.force_n[k][h],initial[n].input.accepted.controlled_hourglass.force_n[k][h]);
    EXPECT_EQ(rows[n].input.accepted.material.internal_energy_density_j_m3,initial[n].input.accepted.material.internal_energy_density_j_m3);}
  ASSERT_EQ(rows[2].status,s::ForceStatus::Success);Compare(x,rows[2].output,Native(NativeHistory(x),x));
  for(auto& r:rows)r.input=x;Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());for(const auto& r:rows){ASSERT_EQ(r.status,s::ForceStatus::Success);Compare(x,r.output,Native(NativeHistory(x),x));}
}
} // namespace h24_test
