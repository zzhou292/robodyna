#include "GroupKickFixture.h"
#include <cuda_runtime.h>
namespace rigid_observation_test {
namespace {
struct Packet {
  fe::NodalRigidGroupProperties group;
  fe::NodalRigidGroupMember metric[Count];
  Motion before[Count],after[Count]; rigid::Wrench applied[Count],reaction[Count];
  fe::NodalRigidGroupState before_group,after_group; Phase before_phase,after_phase; double kick_dt;
};
struct Result { rigid::ObservationReport report; rigid::GroupKickObservation value; };
__global__ void Observe(const Packet* packets,Result* results) {
  for(unsigned i=0;i<3;++i) {
    const auto& p=packets[i];
    const rigid::GroupKickInput input{{&p.group,p.metric,Count},p.before,p.after,p.before_group,p.after_group,
      p.before_phase,p.after_phase,p.applied,p.reaction,p.kick_dt};
    results[i].report=rigid::ObserveGroupKick(input,results[i].value);
  }
}
}
TEST(NodalRigidObservationCuda,ValueOnlyKickMatchesScalarOracleAndPreservesLateFailureOutputs) {
  int count=0; ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess); ASSERT_GT(count,0);
  KickFixture fixture; ASSERT_TRUE(fixture.Solve()); const auto input=fixture.Input();
  std::array<Packet,3> packets;
  for(auto& p:packets) {
    p.group=*input.metric.group; p.before_group=input.before_group; p.after_group=input.after_group;
    p.before_phase=input.before_phase; p.after_phase=input.after_phase; p.kick_dt=input.kick_dt;
    for(unsigned i=0;i<Count;++i) { p.metric[i]=input.metric.members[i]; p.before[i]=input.before_members[i];
      p.after[i]=input.after_members[i]; p.applied[i]=input.applied[i]; p.reaction[i]=input.reaction[i]; }
  }
  packets[1].reaction[3].couple.z=.125; packets[2].after[3].omega.z=1e308;
  std::array<Result,3> results{}; for(auto& r:results) r.value.aggregate_delta=123;
  const auto sentinel=rigid_step_test::Bytes(results[1].value);
  const auto overflow_sentinel=rigid_step_test::Bytes(results[2].value);
  Packet* device_input=nullptr; Result* device_output=nullptr;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device_input),sizeof(packets)),cudaSuccess);
  const auto allocation=cudaMalloc(reinterpret_cast<void**>(&device_output),sizeof(results));
  if(allocation!=cudaSuccess) { cudaFree(device_input); FAIL()<<cudaGetErrorString(allocation); }
  EXPECT_EQ(cudaMemcpy(device_input,packets.data(),sizeof(packets),cudaMemcpyHostToDevice),cudaSuccess);
  EXPECT_EQ(cudaMemcpy(device_output,results.data(),sizeof(results),cudaMemcpyHostToDevice),cudaSuccess);
  Observe<<<1,1>>>(device_input,device_output);
  const auto launch=cudaGetLastError(),synchronized=cudaDeviceSynchronize();
  const auto copied=cudaMemcpy(results.data(),device_output,sizeof(results),cudaMemcpyDeviceToHost);
  cudaFree(device_output); cudaFree(device_input);
  ASSERT_EQ(launch,cudaSuccess); ASSERT_EQ(synchronized,cudaSuccess); ASSERT_EQ(copied,cudaSuccess);
  ASSERT_TRUE(results[0].report); CompareWork(input,results[0].value);
  EXPECT_EQ(results[1].report.status,Status::KickMismatch); EXPECT_EQ(results[1].report.member,3u);
  EXPECT_EQ(rigid_step_test::Bytes(results[1].value),sentinel);
  EXPECT_EQ(results[2].report.status,Status::NonfiniteResult);
  EXPECT_EQ(rigid_step_test::Bytes(results[2].value),overflow_sentinel);
}
} // namespace rigid_observation_test
