#include "InventoryFixture.h"
#include <gtest/gtest.h>
using namespace candidate_test;
namespace {
__global__ void Nothing() {}
void Check(c::Inventory& inventory,const std::vector<c::Pair>& expected,std::size_t rows) {
  const auto view=inventory.view();ASSERT_TRUE(inventory.IsCurrent(view));ASSERT_EQ(view.pair_count(),expected.size());
  ASSERT_EQ(view.secondary_count(),rows);std::vector<c::Pair> actual(view.pair_count());std::vector<std::uint64_t> offsets(rows+1);
  if(!actual.empty())ASSERT_EQ(cudaMemcpy(actual.data(),view.pairs(),actual.size()*sizeof(c::Pair),cudaMemcpyDeviceToHost),cudaSuccess);
  ASSERT_EQ(cudaMemcpy(offsets.data(),view.secondary_offsets(),offsets.size()*sizeof(std::uint64_t),cudaMemcpyDeviceToHost),cudaSuccess);
  for(std::size_t i=0;i<expected.size();++i){EXPECT_EQ(actual[i].secondary_row,expected[i].secondary_row);EXPECT_EQ(actual[i].main_occurrence,expected[i].main_occurrence);}
  for(std::size_t row=0;row<=rows;++row) {
    const auto first=std::lower_bound(expected.begin(),expected.end(),row,[](c::Pair pair,std::size_t value){return pair.secondary_row<value;});
    EXPECT_EQ(offsets[row],std::size_t(first-expected.begin()));
  }
}
}
TEST(NativeCandidateInventory,CompleteMixedMembershipAndUpdates) {
  Scene scene;c::Inventory inventory;ASSERT_EQ(inventory.Initialize(scene.Source(),Limits(),scene.stream),c::Status::Ok);
  auto current=scene.Current();
  for(unsigned step=0;step<4;++step) {
    current.stamp.attempt=step+1;current.stamp.geometry=step+1;current.stamp.gaps=step+1;
    const auto expected=Reference(scene,current);ASSERT_GT(expected.size(),0u);
    ASSERT_EQ(inventory.Stage(current),c::Status::Ok);Check(inventory,expected,scene.secondaries.size());
    const auto report=inventory.last_report();EXPECT_EQ(report.pairs,expected.size());EXPECT_EQ(report.host_fences,3u);
    for(std::size_t i=0;i<scene.secondaries.size();++i)scene.secondary_gaps[i]+=.03125;
    for(std::size_t i=0;i<scene.ids.size();++i)scene.positions[3*i+2]+=.015625*double(i%2);
  }
}
TEST(NativeCandidateInventory,DenseTasksExactCapacityAndNoTruncation) {
  Scene scene(2,1800);auto current=scene.Current();const auto expected=Reference(scene,current);ASSERT_GT(expected.size(),1u);
  c::Inventory exact;ASSERT_EQ(exact.Initialize(scene.Source(),Limits(expected.size(),64),scene.stream),c::Status::Ok);
  ASSERT_EQ(exact.Stage(current),c::Status::Ok);Check(exact,expected,scene.secondaries.size());
  EXPECT_GT(exact.last_report().tasks,2u);
  c::Inventory short_pairs;ASSERT_EQ(short_pairs.Initialize(scene.Source(),Limits(expected.size()-1,64),scene.stream),c::Status::Ok);
  EXPECT_EQ(short_pairs.Stage(current),c::Status::ResourceLimit);EXPECT_EQ(short_pairs.last_report().pairs,expected.size());
  EXPECT_EQ(short_pairs.view().pairs(),nullptr);
  c::Inventory short_tasks;ASSERT_EQ(short_tasks.Initialize(scene.Source(),Limits(4096,1),scene.stream),c::Status::Ok);
  EXPECT_EQ(short_tasks.Stage(current),c::Status::ResourceLimit);EXPECT_EQ(short_tasks.view().pairs(),nullptr);
}
TEST(NativeCandidateInventory,FailureDiscardRetryExpireOnlyTrialView) {
  Scene scene;c::Inventory accepted,trial;
  ASSERT_EQ(accepted.Initialize(scene.Source(),Limits(),scene.stream),c::Status::Ok);
  ASSERT_EQ(trial.Initialize(scene.Source(),Limits(),scene.stream),c::Status::Ok);
  auto current=scene.Current();ASSERT_EQ(accepted.Stage(current),c::Status::Ok);const auto old=accepted.view();
  ASSERT_EQ(trial.Stage(current),c::Status::Ok);const auto prior=trial.view();
  current.stamp.source.topology=999;EXPECT_EQ(trial.Stage(current),c::Status::StaleReference);
  EXPECT_FALSE(trial.IsCurrent(prior));EXPECT_TRUE(accepted.IsCurrent(old));EXPECT_FALSE(accepted.IsCurrent(prior));
  current=scene.Current();scene.secondary_gaps[0]=NAN;
  EXPECT_EQ(trial.Stage(current),c::Status::InvalidInput);EXPECT_EQ(trial.view().pairs(),nullptr);
  scene.secondary_gaps[0]=.125;ASSERT_EQ(trial.Stage(current),c::Status::Ok);Check(trial,Reference(scene,current),scene.secondaries.size());
  const auto retried=trial.view();trial.Discard();EXPECT_FALSE(trial.IsCurrent(retried));EXPECT_TRUE(accepted.IsCurrent(old));
}
TEST(NativeCandidateInventory,InactiveAndClippedFieldsFollowNativeConsumption) {
  Scene scene;c::Inventory inventory;ASSERT_EQ(inventory.Initialize(scene.Source(),Limits(),scene.stream),c::Status::Ok);
  auto current=scene.Current();
  scene.secondary_stiffness[1]=0;scene.secondary_gaps[1]=NAN;
  const auto skipped=scene.secondaries[1];scene.positions[3*skipped]=NAN;scene.velocities[3*skipped]=NAN;
  scene.main_stiffness[1]=0;scene.main_gaps[1]=NAN;scene.curvature[1]=NAN;
  const auto clipped=scene.secondaries[6];scene.positions[3*clipped]=1000.;scene.secondary_gaps[6]=NAN;scene.velocities[3*clipped]=NAN;
  ASSERT_EQ(inventory.Stage(current),c::Status::Ok);Check(inventory,Reference(scene,current),scene.secondaries.size());
  scene.secondary_stiffness[1]=-1.;EXPECT_EQ(inventory.Stage(current),c::Status::InvalidInput);
}
TEST(NativeCandidateInventory,NativeAndSiBorrowedFieldsHaveSameInventory) {
  Scene scene;auto native=scene.Current();const auto expected=Reference(scene,native);c::Inventory inventory;
  ASSERT_EQ(inventory.Initialize(scene.Source(c::InputUnits::Si),Limits(),scene.stream),c::Status::Ok);
  for(std::size_t i=0;i<scene.positions.size;++i){scene.positions[i]*=.001;scene.velocities[i]*=.001;}
  for(std::size_t i=0;i<scene.secondary_gaps.size;++i)scene.secondary_gaps[i]*=.001;
  for(std::size_t i=0;i<scene.main_gaps.size;++i){scene.main_gaps[i]*=.001;scene.curvature[i]*=.001;}
  auto si=scene.Current();si.margin*=.001;si.stored_motion*=.001;si.domain.minimum={-.1,-.1,-.1};si.domain.maximum={.1,.1,.1};
  ASSERT_EQ(inventory.Stage(si),c::Status::Ok);Check(inventory,expected,scene.secondaries.size());
}
TEST(NativeCandidateInventory,EmptyRolesAndEmptyActivityProduceCompleteOffsets) {
  for(unsigned secondaries:{0u,7u}) {
    Scene scene(0,secondaries);c::Inventory inventory;ASSERT_EQ(inventory.Initialize(scene.Source(),Limits(),scene.stream),c::Status::Ok);
    ASSERT_EQ(inventory.Stage(scene.Current()),c::Status::Ok);Check(inventory,{},secondaries);
  }
  Scene scene;c::Inventory inventory;ASSERT_EQ(inventory.Initialize(scene.Source(),Limits(),scene.stream),c::Status::Ok);
  for(std::size_t i=0;i<scene.secondary_stiffness.size;++i)scene.secondary_stiffness[i]=0.;
  ASSERT_EQ(inventory.Stage(scene.Current()),c::Status::Ok);Check(inventory,{},scene.secondaries.size());
}
TEST(NativeCandidateInventory,SourceLimitsMalformedRolesAndBorrowedAliasReject) {
  Scene scene;c::Inventory inventory;c::Forecast forecast;
  auto source=scene.Source();auto limits=Limits();limits.max_device_bytes=1;
  EXPECT_EQ(c::Inventory::Preflight(source,limits,forecast),c::Status::ResourceLimit);
  source.edge_mode=1;EXPECT_EQ(inventory.Initialize(source,Limits(),scene.stream),c::Status::UnsupportedProfile);
  source=scene.Source();scene.ids[1]=scene.ids[0];EXPECT_EQ(inventory.Initialize(source,Limits(),scene.stream),c::Status::InvalidInput);
  scene.ids[1]+=1;ASSERT_EQ(inventory.Initialize(source,Limits(),scene.stream),c::Status::Ok);
  auto current=scene.Current();ASSERT_EQ(inventory.Stage(current),c::Status::Ok);const auto view=inventory.view();
  current.main_gaps=reinterpret_cast<const double*>(view.secondary_offsets());
  EXPECT_EQ(inventory.Stage(current),c::Status::InvalidInput);EXPECT_FALSE(inventory.IsCurrent(view));
}

TEST(NativeCandidateInventory,DevicePoisonCannotTriggerCpuRetry) {
  Scene scene;c::Inventory inventory;ASSERT_EQ(inventory.Initialize(scene.Source(),Limits(),scene.stream),c::Status::Ok);
  ASSERT_EQ(inventory.Stage(scene.Current()),c::Status::Ok);const auto prior=inventory.view();
  Nothing<<<0,32,0,scene.stream>>>();
  EXPECT_EQ(inventory.Stage(scene.Current()),c::Status::DeviceFailure);EXPECT_FALSE(inventory.IsCurrent(prior));
  EXPECT_EQ(inventory.Stage(scene.Current()),c::Status::Unusable);EXPECT_EQ(inventory.view().pairs(),nullptr);
}
