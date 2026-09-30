// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <cuda_runtime.h>
namespace h24_full_test {
struct Row{c::Reference reference;c::History accepted;s::PrescribedInterval interval;c::Scratch scratch;c::Result output;s::ForceStatus status;bool initialization=false;};
__global__ void FullPrepare(Row* rows,unsigned count){unsigned i=threadIdx.x;if(i>=count)return;auto& r=rows[i];r.status=r.initialization?c::PrepareInitial(r.reference,{},r.scratch):c::PrepareCandidate(r.reference,r.accepted,r.interval,r.scratch);}
__global__ void FullComplete(Row* rows,unsigned count,bool accept){unsigned i=threadIdx.x;if(i>=count)return;auto& r=rows[i];if(r.status!=s::ForceStatus::Success)return;r.status=c::Complete(r.scratch,r.scratch.activity.triggers_native_batch!=0,r.output);if(accept&&r.status==s::ForceStatus::Success)r.accepted=r.output.proposed_history;}
class H24ControlledForceCuda:public ::testing::Test{protected:Row* device=nullptr;
 void SetUp()override{int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);ASSERT_EQ(cudaSetDevice(0),cudaSuccess);ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device),4*sizeof(Row)),cudaSuccess);}
 void TearDown()override{if(device)EXPECT_EQ(cudaFree(device),cudaSuccess);}
 void Run(std::vector<Row>& rows,bool accept=true){ASSERT_LE(rows.size(),4);ASSERT_EQ(cudaMemcpy(device,rows.data(),rows.size()*sizeof(Row),cudaMemcpyHostToDevice),cudaSuccess);FullPrepare<<<1,4>>>(device,rows.size());ASSERT_EQ(cudaGetLastError(),cudaSuccess);FullComplete<<<1,4>>>(device,rows.size(),accept);ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaMemcpy(rows.data(),device,rows.size()*sizeof(Row),cudaMemcpyDeviceToHost),cudaSuccess);}
};
TEST_F(H24ControlledForceCuda, NativeHistoryCarryInBothUnitsAndExactRepeat){std::vector<q::Case> cases{Case(false),Case(true)};std::vector<NativeState> native;std::vector<Row> rows(2);
 for(unsigned i=0;i<2;++i){native.emplace_back(cases[i]);rows[i].reference=Reference(cases[i]);rows[i].initialization=true;cases[i].interval.dt_s=0;cases[i].interval.sample_index=0;}
 Run(rows);ASSERT_FALSE(HasFailure());for(unsigned i=0;i<2;++i){ASSERT_EQ(rows[i].status,s::ForceStatus::Success);const auto n=native[i].Step(cases[i]);Compare(cases[i],rows[i].scratch,rows[i].output,n);native[i].Accept(n);rows[i].initialization=false;cases[i].interval.dt_s=1e-6;cases[i].interval.sample_index=1;}
 for(unsigned step=0;step<32;++step){SCOPED_TRACE(step);for(unsigned i=0;i<2;++i){q::Move(cases[i],(step%8<4?1.:-1.)*(.1+.01*step));rows[i].interval=cases[i].interval;}
  const auto before=rows;Run(rows);ASSERT_FALSE(HasFailure());for(unsigned i=0;i<2;++i){SCOPED_TRACE(i);ASSERT_EQ(rows[i].status,s::ForceStatus::Success);const auto n=native[i].Step(cases[i]);Compare(cases[i],rows[i].scratch,rows[i].output,n);native[i].Accept(n);cases[i].interval.base_time_s+=cases[i].interval.dt_s;++cases[i].interval.sample_index;}
  ASSERT_FALSE(HasFailure());auto repeat=before;Run(repeat);ASSERT_FALSE(HasFailure());for(unsigned i=0;i<2;++i)EXPECT_EQ(Values(rows[i].output),Values(repeat[i].output));
 }
}
TEST_F(H24ControlledForceCuda, MixedInvalidRowPreservesHistoryAndResult){auto x=Case(true);const auto ref=Reference(x);NativeState native(x);auto initial=Initial(x,ref,native);ASSERT_FALSE(HasFailure());std::vector<Row> rows(2);
 for(auto& r:rows){r.reference=ref;r.accepted=initial.proposed_history;r.output=initial;r.interval=x.interval;}
 rows[0].interval.velocity_m_s[1].x=std::numeric_limits<double>::quiet_NaN();const auto before=rows;Run(rows);ASSERT_FALSE(HasFailure());EXPECT_NE(rows[0].status,s::ForceStatus::Success);EXPECT_EQ(Values(rows[0].output),Values(before[0].output));EXPECT_EQ(History(rows[0].accepted),History(before[0].accepted));ASSERT_EQ(rows[1].status,s::ForceStatus::Success);
 rows[0].interval=x.interval;Run(rows);ASSERT_FALSE(HasFailure());EXPECT_EQ(rows[0].status,s::ForceStatus::Success);
}
} // namespace h24_full_test
