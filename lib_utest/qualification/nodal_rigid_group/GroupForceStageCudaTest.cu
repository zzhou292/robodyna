#include "GroupForceStageFixture.h"
#include <cuda_runtime.h>
namespace rigid_observation_test {
namespace {
struct Packet {
  fe::NodalRigidGroupProperties group;
  fe::NodalRigidGroupMember metric[Count];Motion before[Count];
  rigid::ForceStageAcceleration acceleration[Count],primary_acceleration;
  Motion primary;Matrix axes;rigid::ForceStageObservationPhase phase;
};
struct Result {rigid::ObservationReport report;rigid::GroupForceStageKineticObservation value;};
__global__ void Observe(const Packet* packets,Result* results) {
  for(unsigned i=0;i<3;++i) {const auto& p=packets[i];
    const rigid::GroupForceStageKineticInput in{{&p.group,p.metric,Count},p.before,p.acceleration,p.primary,
      p.primary_acceleration,p.axes,p.phase};
    results[i].report=rigid::ObserveGroupForceStageKinetic(in,results[i].value);
  }
}
}
TEST(NodalRigidForceStageCuda,NonzeroForceStageAndLateFailureMatchQualifiedValueScope) {
  int count=0;ASSERT_EQ(cudaGetDeviceCount(&count),cudaSuccess);ASSERT_GT(count,0);
  ForceStageFixture f;ASSERT_TRUE(f.kick.Solve());f.kick.Carry(1);ASSERT_TRUE(f.kick.Solve());const auto in=f.Input();
  std::array<Packet,3> packets{};std::array<Result,3> results{};
  for(auto& p:packets) {
    p.group=*in.metric.group;p.primary=in.before_primary;p.primary_acceleration=in.primary_acceleration;p.axes=in.force_frame;p.phase=in.phase;
    for(unsigned i=0;i<Count;++i){p.metric[i]=in.metric.members[i];p.before[i]=in.before_members[i];p.acceleration[i]=in.member_acceleration[i];}
  }
  packets[1].phase.durations.kick_dt*=2;packets[2].acceleration[Count-1].rotation.z=1e308;
  for(auto& r:results)r.value.replacement=123;const auto sentinel=rigid_step_test::Bytes(results[1].value);
  Packet* device_input=nullptr;Result* device_output=nullptr;
  ASSERT_EQ(cudaMalloc(reinterpret_cast<void**>(&device_input),sizeof(packets)),cudaSuccess);
  const auto allocation=cudaMalloc(reinterpret_cast<void**>(&device_output),sizeof(results));
  if(allocation!=cudaSuccess){cudaFree(device_input);FAIL()<<cudaGetErrorString(allocation);}
  EXPECT_EQ(cudaMemcpy(device_input,packets.data(),sizeof(packets),cudaMemcpyHostToDevice),cudaSuccess);
  EXPECT_EQ(cudaMemcpy(device_output,results.data(),sizeof(results),cudaMemcpyHostToDevice),cudaSuccess);
  Observe<<<1,1>>>(device_input,device_output);const auto launch=cudaGetLastError(),sync=cudaDeviceSynchronize();
  const auto copy=cudaMemcpy(results.data(),device_output,sizeof(results),cudaMemcpyDeviceToHost);cudaFree(device_output);cudaFree(device_input);
  ASSERT_EQ(launch,cudaSuccess);ASSERT_EQ(sync,cudaSuccess);ASSERT_EQ(copy,cudaSuccess);
  ASSERT_TRUE(results[0].report);CheckForceStageOracle(in,results[0].value);
  EXPECT_EQ(results[1].report.status,Status::UnsupportedPhase);EXPECT_EQ(rigid_step_test::Bytes(results[1].value),sentinel);
  EXPECT_EQ(results[2].report.status,Status::NonfiniteResult);EXPECT_EQ(rigid_step_test::Bytes(results[2].value),sentinel);
}
} // namespace rigid_observation_test
