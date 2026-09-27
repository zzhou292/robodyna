// SPDX-License-Identifier: AGPL-3.0-or-later
#include "WorkingSupport.h"
#include <cuda_runtime.h>
namespace h24_test {
struct WorkingRow {c::WorkingReference reference;c::HistoryValues accepted;s::PrescribedInterval interval;c::WorkingResult output;s::ForceStatus status;};
__global__ void WorkingKernel(WorkingRow* rows,unsigned count,bool accept) {
  const unsigned i=threadIdx.x;if(i>=count)return;auto& r=rows[i];
  r.status=c::EvaluateWorking(r.reference,r.accepted,r.interval,false,r.output);
  if(accept&&r.status==s::ForceStatus::Success)r.accepted=r.output.stage.proposed_values;
}
class H24WorkingCuda:public ::testing::Test {
 protected:
  WorkingRow* device=nullptr;
  void SetUp()override{int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);ASSERT_EQ(cudaSetDevice(0),cudaSuccess);ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device),4*sizeof(WorkingRow)),cudaSuccess);}
  void TearDown()override{if(device)EXPECT_EQ(cudaFree(device),cudaSuccess);}
  void Run(std::vector<WorkingRow>& rows,bool accept){ASSERT_LE(rows.size(),4);ASSERT_EQ(cudaMemcpy(device,rows.data(),rows.size()*sizeof(WorkingRow),cudaMemcpyHostToDevice),cudaSuccess);
    WorkingKernel<<<1,4>>>(device,rows.size(),accept);ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaMemcpy(rows.data(),device,rows.size()*sizeof(WorkingRow),cudaMemcpyDeviceToHost),cudaSuccess);}
};
TEST_F(H24WorkingCuda, NativeMillimetreCarryAndDeterministicValues) {
  auto x=MillimetreCase();auto native_history=MillimetreNativeHistory(x);std::vector<WorkingRow> rows(1);rows[0].reference=WorkingReference(x);rows[0].accepted=x.accepted;
  for(unsigned step=0;step<32;++step){SCOPED_TRACE(step);Move(x,(step%8<4?1.:-1.)*(.1+.01*step));rows[0].interval=x.interval;
    const auto pre=rows;Run(rows,true);ASSERT_FALSE(HasFailure());ASSERT_EQ(rows[0].status,s::ForceStatus::Success);
    const auto n=MillimetreNative(native_history,x);CompareWorking(x,rows[0].output,n);ASSERT_FALSE(HasFailure());
    auto repeat=pre;Run(repeat,true);ASSERT_FALSE(HasFailure());EXPECT_EQ(Values(repeat[0].output),Values(rows[0].output));
    native_history.values=n.carried;x.interval.base_time_s+=x.interval.dt_s;++x.interval.sample_index;
  }
}
TEST_F(H24WorkingCuda, MillimetreFloorAndMixedFailureAtomicity) {
  auto floor=MillimetreCase(solid24_test::FloorControl(s::WorkingLengthUnit::Millimetre,.5));floor.accepted.material.internal_energy_density_j_m3=123.25;
  auto regular=MillimetreCase();Move(regular);std::vector<WorkingRow> rows(2);
  rows[0].reference=WorkingReference(floor);rows[0].accepted=floor.accepted;rows[0].interval=floor.interval;
  rows[1].reference=WorkingReference(regular);rows[1].accepted=regular.accepted;rows[1].interval=regular.interval;
  Run(rows,false);ASSERT_FALSE(HasFailure());for(const auto& r:rows)ASSERT_EQ(r.status,s::ForceStatus::Success);
  CompareWorking(floor,rows[0].output,MillimetreNative(MillimetreNativeHistory(floor),floor));const auto before=Values(rows[1].output);
  rows[1].accepted.controlled_hourglass.force_n[0][2]=std::numeric_limits<double>::max();const auto saved=rows[1].accepted;
  Run(rows,true);ASSERT_FALSE(HasFailure());EXPECT_NE(rows[1].status,s::ForceStatus::Success);EXPECT_EQ(Values(rows[1].output),before);
  EXPECT_EQ(rows[1].accepted.controlled_hourglass.force_n[0][2],saved.controlled_hourglass.force_n[0][2]);
}
} // namespace h24_test
