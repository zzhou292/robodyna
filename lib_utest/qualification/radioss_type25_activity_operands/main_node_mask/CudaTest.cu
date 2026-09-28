// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Native.h"
namespace activity_operands_test {
void Exact(const Saved&,const Saved&);
TEST(MainNodeRoleCuda, SharedGlobalSupportAndKeepDisconnectedMatchOriginalChkmsr) {
  GpuFixture f(false,true,false,false,false,false,true);ASSERT_TRUE(f.Initialize());
  const auto initial=Read(f,0);ASSERT_EQ(initial.main_activity.size(),f.source.selection.node_count);
  const auto first=f.operands.Stage(0,f.Activity({1,1},{0,1}));ASSERT_TRUE(Good(first.report));
  const auto one=Read(f,first.staged_slot);
  EXPECT_EQ(one.main_activity,NativeMainMask(f,Mesh(f,{1,1},{0,1},1,1),initial.main_activity));
  EXPECT_EQ(one.main_activity,initial.main_activity);
  const auto next=f.operands.Stage(first.staged_slot,f.Activity({0,1},{0,0},1,1,2));ASSERT_TRUE(Good(next.report));
  const auto two=Read(f,next.staged_slot);
  EXPECT_EQ(two.main_activity,NativeMainMask(f,Mesh(f,{0,1},{0,0},1,1),one.main_activity));
  EXPECT_GT(std::count(two.main_activity.begin(),two.main_activity.end(),0),0);
  EXPECT_EQ(two.secondary,initial.secondary);EXPECT_EQ(next.orphan_secondaries,0u);
}
TEST(MainNodeRoleCuda, MaskOnlyRetirementStillChangesSourceContent) {
  GpuFixture f(false,true,false,false,false,false,true);
  for(auto& main:f.physical.mains){main.coefficient=0;for(auto& neighbor:main.neighbors)neighbor=0;}
  ASSERT_TRUE(f.Initialize());const auto before=Read(f,0);
  const auto staged=f.operands.Stage(0,f.Activity({1,1},{0,0},1,0));ASSERT_TRUE(Good(staged.report));
  const auto after=Read(f,staged.staged_slot);
  EXPECT_NE(after.main_activity,before.main_activity);EXPECT_TRUE(staged.changed);
  EXPECT_EQ(staged.removed_mains,0u);EXPECT_EQ(staged.orphan_secondaries,0u);
  EXPECT_EQ(after.secondary,before.secondary);EXPECT_EQ(after.connected,before.connected);
  for(std::size_t i=0;i<after.mains.size();++i)EXPECT_EQ(after.mains[i].coefficient,before.mains[i].coefficient);
  EXPECT_EQ(after.main_activity,NativeMainMask(f,Mesh(f,{1,1},{0,0},1,0),before.main_activity));
}
TEST(MainNodeRoleCuda, NonMainNonSecondaryOrphansCannotCreateSpuriousRebuilds) {
  GpuFixture f(false,true,false,false,true,true);ASSERT_TRUE(f.Initialize());const auto before=Read(f,0);
  const auto mesh=Mesh(f,{1,1},{1,1},1,0);const auto tags=native::Tag(mesh);
  const auto original_tags=native::Tag(Mesh(f,{1,1},{1,1},1,1));
  std::vector<std::uint8_t> member(before.main_activity.size());
  for(std::size_t i=0;i<f.source.selection.main_count;++i)
    for(auto node:f.source.selection.mains[i].nodes)member.at(node)=1;
  std::size_t new_nonmember_orphans=0;
  for(std::size_t node=0;node<member.size();++node)
    if(original_tags.active_nodes[node] && !tags.active_nodes[node]) {
      ASSERT_EQ(member[node],0);++new_nonmember_orphans;
    }
  ASSERT_GT(new_nonmember_orphans,0u);
  const auto staged=f.operands.Stage(0,f.Activity({1,1},{1,1},1,0));ASSERT_TRUE(Good(staged.report));
  EXPECT_FALSE(staged.changed);const auto after=Read(f,staged.staged_slot);Exact(before,after);
  EXPECT_EQ(after.main_activity,NativeMainMask(f,mesh,before.main_activity));
}
TEST(MainNodeRoleCuda, PublishedMaskSurvivesDiscardFailedStageAndExactRetry) {
  GpuFixture f(false);ASSERT_TRUE(f.Initialize());const auto before=Read(f,0);
  const auto first=f.operands.Stage(0,f.Activity({1,1},{0,0},1,0));ASSERT_TRUE(Good(first.report));
  const auto after=Read(f,first.staged_slot);ASSERT_NE(before.main_activity,after.main_activity);
  EXPECT_NE(f.operands.view(0).main_node_activity,f.operands.view(1).main_node_activity);
  EXPECT_FALSE(f.operands.OutputDisjoint(f.operands.view(0).main_node_activity,before.main_activity.size()));
  f.operands.DiscardStaged(0,1);EXPECT_EQ(f.operands.view(1).main_node_activity,nullptr);EXPECT_EQ(f.operands.view(1).node_count,0u);
  Exact(before,Read(f,0));
  auto bad=f.Activity({1,1},{0,0},1,2,2);
  EXPECT_EQ(f.operands.Stage(0,bad).report.status,n::TransactionStatus::ActivityChange);Exact(before,Read(f,0));
  const auto retry=f.operands.Stage(0,f.Activity({1,1},{0,0},1,0,3));ASSERT_TRUE(Good(retry.report));Exact(after,Read(f,retry.staged_slot));
  const auto stable=f.operands.Stage(retry.staged_slot,f.Activity({0,0},{0,0},0,0,4));ASSERT_TRUE(Good(stable.report));
  EXPECT_FALSE(stable.changed);Exact(after,Read(f,stable.staged_slot));
}
TEST(MainNodeRoleCuda, WallRetentionAndExactMaskReservationsRemainExplicit) {
  GpuFixture f(false,false);const auto shape=f.Shape();
  const auto forecast=a::State::Preflight(f.plan,f.source,nullptr,f.units,shape);ASSERT_TRUE(Good(forecast.report));
  const auto nodes=f.source.selection.node_count;
  EXPECT_EQ(forecast.main_node_mask_device_bytes,2*nodes);EXPECT_EQ(forecast.main_membership_device_bytes,nodes);
  EXPECT_EQ(forecast.main_node_upload_host_bytes,2*nodes);
  a::Limits limits;limits.max_device_bytes=forecast.owned_device_bytes;limits.max_host_bytes=forecast.owned_host_bytes;
  limits.max_startup_host_bytes=forecast.startup_host_bytes;
  EXPECT_TRUE(Good(a::State::Preflight(f.plan,f.source,nullptr,f.units,shape,limits).report));
  --limits.max_device_bytes;EXPECT_EQ(a::State::Preflight(f.plan,f.source,nullptr,f.units,shape,limits).report.status,n::TransactionStatus::ResourceLimit);
  ++limits.max_device_bytes;--limits.max_startup_host_bytes;
  EXPECT_EQ(a::State::Preflight(f.plan,f.source,nullptr,f.units,shape,limits).report.status,n::TransactionStatus::ResourceLimit);
  ASSERT_TRUE(f.Initialize());const auto old=Read(f,0);
  const auto staged=f.operands.Stage(0,f.Activity({1,1},{0,0},1,0));ASSERT_TRUE(Good(staged.report));
  EXPECT_FALSE(staged.changed);const auto current=Read(f,staged.staged_slot);Exact(old,current);
  EXPECT_EQ(std::count(current.main_activity.begin(),current.main_activity.end(),1),nodes);
}
}
