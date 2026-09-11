#include "VehicleCudaFixture.h"
#include <limits>

namespace type25_batch_test {
using Type25VehicleCuda=vehicle::VehicleOwnerCuda;
TEST_F(Type25VehicleCuda,MaximumFinalEndpointCandidateRollbackReadbackCapsAndExactRetry) {
  VehicleRig rig;ASSERT_TRUE(rig.Initialize());
  std::vector<spring::Evaluation> before,after,first(4096),retry(4096);
  spring::BatchDiagnostics initial,observed,clean,good;ASSERT_TRUE(rig.Read(before,initial));
  const auto allocation=rig.batch.allocations();const auto host=rig.batch.host_bytes();const auto accepted=rig.owner.accepted();
  EXPECT_EQ(initial.element_count,4096u);EXPECT_GT(initial.minimum_native_dt,VehicleStep);
  vehicle::Fields base(rig.initial.n),restored(rig.initial.n);fe::NodalStamp stamp;
  ASSERT_EQ(rig.owner.CopyAccepted(base.buffer(),&stamp).status,fe::NodalStatus::Ok);
  fe::NodalTrialToken token;fe::NodalPreparedView prepared;ASSERT_TRUE(rig.Prepare(token,prepared));
  ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&clean).status,spring::BatchStatus::Success);
  first.back().critical_dt_s=771;
  EXPECT_EQ(rig.batch.CopyPreparedResults(clean,first.data(),4095).status,spring::BatchStatus::ResourceLimit);
  EXPECT_EQ(first.back().critical_dt_s,771);
  ASSERT_EQ(rig.batch.CopyPreparedResults(clean,first.data(),first.size()).status,spring::BatchStatus::Success);
  VehicleAgainstHost(rig,token,prepared,before,first);
  EXPECT_GT(tl::math::fixed3::Norm(first.back().endpoints[0].force_N),0.);
  EXPECT_GT(tl::math::fixed3::Norm(first.back().endpoints[0].couple_Nm),0.);
  rig.Discard();ASSERT_TRUE(rig.Prepare(token,prepared));
  const double bad=std::numeric_limits<double>::quiet_NaN();
  ASSERT_EQ(cudaMemcpyAsync(const_cast<double*>(prepared.kinematics.position_xyz)+3*rig.initial.n-2,
    &bad,sizeof(bad),cudaMemcpyHostToDevice,prepared.stream),cudaSuccess);
  ASSERT_EQ(cudaStreamSynchronize(prepared.stream),cudaSuccess);observed.attempt=998;
  const auto rejected=rig.batch.EvaluateCandidate(rig.owner,token,prepared,&observed);
  EXPECT_EQ(rejected.status,spring::BatchStatus::ElementFailure);EXPECT_EQ(rejected.element,4095u);EXPECT_EQ(observed.attempt,998u);
  ASSERT_TRUE(rig.Read(after,observed));EXPECT_TRUE(spring::batch_detail::SameDiagnostics(initial,observed));
  for(std::size_t e=0;e<before.size();++e)ExactVehicle(before[e],after[e]);
  rig.Discard();ASSERT_EQ(rig.owner.CopyAccepted(restored.buffer(),&stamp).status,fe::NodalStatus::Ok);
  vehicle::SameFields(base,restored);EXPECT_TRUE(fe::trial_identity::SameStamp(accepted,stamp));
  ASSERT_TRUE(rig.Prepare(token,prepared));ASSERT_EQ(rig.batch.EvaluateCandidate(rig.owner,token,prepared,&good).status,spring::BatchStatus::Success);
  ASSERT_EQ(rig.batch.CopyPreparedResults(good,retry.data(),retry.size()).status,spring::BatchStatus::Success);
  for(std::size_t e=0;e<retry.size();++e)ExactVehicle(first[e],retry[e]);
  EXPECT_GT(good.attempt,clean.attempt);EXPECT_EQ(good.time,clean.time);
  EXPECT_EQ(rig.batch.allocations().device_allocations,1u);EXPECT_EQ(rig.batch.allocations().device_bytes,allocation.device_bytes);
  EXPECT_EQ(rig.batch.host_bytes(),host);rig.Discard();
  observed.attempt=336;retry.back().critical_dt_s=779;
  EXPECT_EQ(rig.batch.CopyAcceptedResults(accepted,retry.data(),4095,&observed).status,spring::BatchStatus::ResourceLimit);
  EXPECT_EQ(observed.attempt,336u);EXPECT_EQ(retry.back().critical_dt_s,779);
}
TEST_F(Type25VehicleCuda,ExplicitResidentAndSharedStorageBudgetsRejectBeforeAllocation) {
  VehicleRig rig(2052,1025);ASSERT_TRUE(rig.Initialize());const auto config=rig.input.Config(rig.owner.accepted());
  spring::Batch legacy;EXPECT_EQ(legacy.InitializeJoined(config,rig.input.model,rig.input.mass).status,spring::BatchStatus::ResourceLimit);
  EXPECT_EQ(legacy.allocations().device_bytes,0u);
  for(unsigned mode=0;mode<4;++mode) {
    auto bad=config;spring::Batch candidate;
    if(mode==0)bad.max_device_bytes=rig.batch.allocations().device_bytes-1;
    if(mode==1)bad.max_host_bytes=rig.batch.host_bytes()-1;
    if(mode==2)bad.element_count=4097;
    if(mode==3)bad.owner.node_count=524289;
    EXPECT_EQ(candidate.InitializeJoined(bad,rig.input.model,rig.input.mass,spring::CapacityProfile::Vehicle).status,spring::BatchStatus::ResourceLimit);
    EXPECT_EQ(candidate.allocations().device_allocations,0u);EXPECT_EQ(candidate.host_bytes(),0u);
  }
  spring::Model copy(rig.input.model),equal;ASSERT_TRUE(equal.Initialize(rig.input.source.Input()));
  auto exact=config;exact.max_host_bytes=rig.batch.host_bytes();exact.max_device_bytes=rig.batch.allocations().device_bytes;
  spring::Batch shared,independent;
  ASSERT_EQ(shared.InitializeJoined(exact,copy,rig.input.mass,spring::CapacityProfile::Vehicle).status,spring::BatchStatus::Success);
  EXPECT_EQ(shared.host_bytes(),rig.batch.host_bytes());
  EXPECT_EQ(independent.InitializeJoined(exact,equal,rig.input.mass,spring::CapacityProfile::Vehicle).status,spring::BatchStatus::ResourceLimit);
  EXPECT_EQ(independent.allocations().device_bytes,0u);
  exact.max_host_bytes+=equal.owned_payload_bytes()-sizeof(spring::Model);
  ASSERT_EQ(independent.InitializeJoined(exact,equal,rig.input.mass,spring::CapacityProfile::Vehicle).status,spring::BatchStatus::Success);
  EXPECT_EQ(independent.host_bytes(),exact.max_host_bytes);
}
} // namespace type25_batch_test
