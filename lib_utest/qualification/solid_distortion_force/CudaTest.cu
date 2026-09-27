// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <cuda_runtime.h>
namespace distortion_force_test {
struct Packet {d::PreparedForceValues values;d::ForceResult result;int batch=0;d::Status status=d::Status::InvalidInput;};
__global__ void Evaluate(Packet* values,unsigned count) {
  const unsigned i=blockIdx.x*blockDim.x+threadIdx.x;
  if(i<count)values[i].status=d::EvaluateForce(values[i].values,values[i].batch!=0,values[i].result);
}
__global__ void ClassifyPair(Packet* values,int* batch) {
  // Dedicated numerical witness: authentic two-row native packet, independent
  // of block size. Actual owner batching must authenticate native NEL grouping.
  if(threadIdx.x==0) {
    int combined=0;for(unsigned n=0;n<2;++n){d::DampingActivity a;
      if(d::ClassifyDamping(values[n].values,a)==d::Status::Success)combined|=a.triggers_native_batch;}
    *batch=combined;for(unsigned n=0;n<2;++n)values[n].batch=combined;
  }
}
class DistortionForceCuda:public ::testing::Test {
 protected:
  Packet* device=nullptr;int* gate=nullptr;
  void SetUp() override {int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);ASSERT_EQ(cudaSetDevice(0),cudaSuccess);
    ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device),128*sizeof(Packet)),cudaSuccess);ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&gate),sizeof(int)),cudaSuccess);}
  void TearDown() override {if(device)EXPECT_EQ(cudaFree(device),cudaSuccess);if(gate)EXPECT_EQ(cudaFree(gate),cudaSuccess);}
  void Upload(const std::vector<Packet>& rows) {ASSERT_LE(rows.size(),128);ASSERT_EQ(cudaMemcpy(device,rows.data(),rows.size()*sizeof(Packet),cudaMemcpyHostToDevice),cudaSuccess);}
  void Run(std::vector<Packet>& rows) {Evaluate<<<2,64>>>(device,rows.size());ASSERT_EQ(cudaGetLastError(),cudaSuccess);
    ASSERT_EQ(cudaMemcpy(rows.data(),device,rows.size()*sizeof(Packet),cudaMemcpyDeviceToHost),cudaSuccess);}
};
TEST_F(DistortionForceCuda, SiAndOriginalMillimetreValuesMatchNativeAndRepeatExactly) {
  const auto cases=Cases();std::vector<Packet> rows;
  for(const auto& c:cases){auto p=Prepare(c);rows.push_back({p,{},Batch(p)?1:0});}
  Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());const auto first=rows;
  for(unsigned n=0;n<rows.size();++n){SCOPED_TRACE(n);ASSERT_EQ(rows[n].status,d::Status::Success);Compare(rows[n].result,Native({cases[n]})[0]);}
  Run(rows);ASSERT_FALSE(HasFailure());for(unsigned n=0;n<rows.size();++n)Same(rows[n].result,first[n].result);
}
TEST_F(DistortionForceCuda, AuthenticatedTwoRowBatchOrDampsZeroMeanBuckledRow) {
  auto zero=Base(true),trigger=zero;zero.units=trigger.units={.001,1000,1};
  for(unsigned n=0;n<8;++n){zero.input.velocity_m_s[n]={0,0,n%2?1.:-1.};trigger.input.velocity_m_s[n]={0,0,(n%2?1.:-1.)+.01};}
  std::vector<Packet> rows{{Prepare(zero),{},0},{Prepare(trigger),{},0}};Upload(rows);ASSERT_FALSE(HasFailure());
  ClassifyPair<<<1,32>>>(device,gate);ASSERT_EQ(cudaGetLastError(),cudaSuccess);Run(rows);ASSERT_FALSE(HasFailure());
  const auto expected=Native({zero,trigger});
  for(unsigned n=0;n<2;++n){ASSERT_EQ(rows[n].status,d::Status::Success);EXPECT_EQ(rows[n].batch,1);Compare(rows[n].result,expected[n]);EXPECT_EQ(rows[n].result.damping_applied,1);}
}
TEST_F(DistortionForceCuda, LoadingUnloadingKeepsNativeCumulativeAndSeparateWork) {
  auto c=Folded();c.units={.001,1000,1};std::vector<Packet> rows(1);
  for(unsigned step=0;step<24;++step) {
    for(unsigned n=0;n<8;++n)c.input.velocity_m_s[n]={0,0,(n%2?.02:-.02)*(step%2?-1.:1.)};
    rows[0]={Prepare(c),{},0};Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());
    ASSERT_EQ(rows[0].status,d::Status::Success);Compare(rows[0].result,Native({c})[0]);
    c.input.distortion_energy_j=rows[0].result.distortion_energy_j;
  }
}
TEST_F(DistortionForceCuda, DegenerateLeafMatchesNativeWithoutExtraVolumeGuard) {
  std::vector<Packet> rows;
  for(auto units:{d::UnitScale{1,1,1},d::UnitScale{.001,1000,1}}) {
    auto c=Folded();c.units=units;auto values=Prepare(c);
    for(auto& point:values.input.position)point={};rows.push_back({values,{},Batch(values)?1:0});
  }
  Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());
  for(const auto& row:rows){ASSERT_EQ(row.status,d::Status::Success);Compare(row.result,NativePrepared(row.values));}
}
TEST_F(DistortionForceCuda, MixedFailuresPreserveOutputsAndRepair) {
  const auto good=Prepare(Folded());const auto sentinel=Check(Folded());std::vector<Packet> rows(4);
  for(auto& row:rows)row={good,sentinel,Batch(good)?1:0};
  rows[0].values.input.position[0].x=std::numeric_limits<double>::quiet_NaN();
  rows[1].values.input.dt=-1;
  rows[2].values.parameters.damping=std::numeric_limits<double>::max();rows[2].values.input.velocity[0].x=4;rows[2].batch=1;
  Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());
  for(unsigned n=0;n<3;++n){EXPECT_NE(rows[n].status,d::Status::Success);Same(rows[n].result,sentinel);}
  ASSERT_EQ(rows[3].status,d::Status::Success);
  for(auto& row:rows)row={good,sentinel,Batch(good)?1:0};Upload(rows);ASSERT_FALSE(HasFailure());Run(rows);ASSERT_FALSE(HasFailure());
  for(const auto& row:rows){ASSERT_EQ(row.status,d::Status::Success);Compare(row.result,Native({Folded()})[0]);}
}
} // namespace distortion_force_test
