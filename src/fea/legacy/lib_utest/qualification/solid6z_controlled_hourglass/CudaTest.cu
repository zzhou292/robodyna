// SPDX-License-Identifier: AGPL-3.0-or-later
#include "TestSupport.h"
#include <cuda_runtime.h>
namespace s6_control_test {
struct Row{Case input;Trial output;s::Status status=s::Status::InvalidInput;};
__global__ void ControlledKernel(Row* rows,unsigned count,bool accept){unsigned i=threadIdx.x;if(i>=count)return;auto& r=rows[i];r.status=Evaluate(r.input,r.output);if(accept&&r.status==s::Status::Success)r.input.accepted=r.output.result.proposed_values;}
class S6ControlledCuda:public ::testing::Test{protected:Row* device=nullptr;
 void SetUp()override{int n=0;ASSERT_EQ(cudaGetDeviceCount(&n),cudaSuccess);ASSERT_GT(n,0);ASSERT_EQ(cudaSetDevice(0),cudaSuccess);ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device),32*sizeof(Row)),cudaSuccess);}
 void TearDown()override{if(device)EXPECT_EQ(cudaFree(device),cudaSuccess);}
 void Run(std::vector<Row>& rows,bool accept=false){ASSERT_LE(rows.size(),32);ASSERT_EQ(cudaMemcpy(device,rows.data(),rows.size()*sizeof(Row),cudaMemcpyHostToDevice),cudaSuccess);ControlledKernel<<<1,32>>>(device,rows.size(),accept);ASSERT_EQ(cudaGetLastError(),cudaSuccess);ASSERT_EQ(cudaMemcpy(rows.data(),device,rows.size()*sizeof(Row),cudaMemcpyDeviceToHost),cudaSuccess);}
};
TEST_F(S6ControlledCuda, NativeStagesAndDeterministicRepeat){std::vector<Row> rows;for(const auto& x:Cases())rows.push_back({x});Run(rows);ASSERT_FALSE(HasFailure());const auto first=rows;
 for(const auto& r:rows){ASSERT_EQ(r.status,s::Status::Success);Compare(r.input,r.output,NativeHistory(r.input).Step(r.input));}Run(rows);ASSERT_FALSE(HasFailure());for(unsigned i=0;i<rows.size();++i)EXPECT_EQ(Values(rows[i].output),Values(first[i].output));}
TEST_F(S6ControlledCuda, DeviceCarried32Intervals){auto x=Base();NativeHistory native(x);std::vector<Row> rows{{x}};
 for(unsigned step=0;step<32;++step){SCOPED_TRACE(step);rows[0].input.interval=legacy::Path(x.reference,step*12);Run(rows,true);ASSERT_FALSE(HasFailure());ASSERT_EQ(rows[0].status,s::Status::Success);const auto n=native.Step(rows[0].input);Compare(rows[0].input,rows[0].output,n);ASSERT_FALSE(HasFailure());native.accepted=n.history;}}
TEST_F(S6ControlledCuda, MixedRejectionAndRepair){auto x=Base();x.interval=legacy::Path(x.reference,10);const auto before=Check(x);std::vector<Row> rows(3);for(auto& r:rows){r.input=x;r.output=before;}
 rows[0].input.accepted.controlled_hourglass.force_n[1][3]=std::numeric_limits<double>::max();rows[1].input.interval.velocity_midpoint_m_s[0].x=std::numeric_limits<double>::infinity();const auto initial=rows;
 Run(rows,true);ASSERT_FALSE(HasFailure());for(unsigned i=0;i<2;++i){EXPECT_NE(rows[i].status,s::Status::Success);EXPECT_EQ(Values(rows[i].output),Values(before));EXPECT_EQ(History(rows[i].input.accepted),History(initial[i].input.accepted));}
 ASSERT_EQ(rows[2].status,s::Status::Success);Compare(x,rows[2].output,NativeHistory(x).Step(x));for(auto& r:rows)r.input=x;Run(rows);ASSERT_FALSE(HasFailure());for(const auto& r:rows){ASSERT_EQ(r.status,s::Status::Success);Compare(x,r.output,NativeHistory(x).Step(x));}}
} // namespace s6_control_test
