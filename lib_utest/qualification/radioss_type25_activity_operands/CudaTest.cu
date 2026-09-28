// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeComparison.h"
#include "lib_src/collision/radioss_type25/activity_operands/Values.h"
namespace activity_operands_test {
void Exact(const Saved& left,const Saved& right) {
  ASSERT_EQ(left.mains.size(),right.mains.size());
  for(std::size_t i=0;i<left.mains.size();++i){
    EXPECT_TRUE(a::detail::Same(left.mains[i].coefficient,right.mains[i].coefficient));
    EXPECT_EQ(left.mains[i].global_id,right.mains[i].global_id);EXPECT_EQ(left.mains[i].segment_type,right.mains[i].segment_type);
    for(unsigned k=0;k<4;++k){EXPECT_EQ(left.mains[i].nodes[k],right.mains[i].nodes[k]);
      EXPECT_EQ(left.mains[i].neighbors[k],right.mains[i].neighbors[k]);
      EXPECT_EQ(left.mains[i].normal_reference[k],right.mains[i].normal_reference[k]);}
  }
  EXPECT_EQ(left.secondary,right.secondary);EXPECT_EQ(left.connected,right.connected);EXPECT_EQ(left.free,right.free);
  EXPECT_EQ(left.main_si,right.main_si);EXPECT_EQ(left.secondary_si,right.secondary_si);EXPECT_EQ(left.normal_coefficients,right.normal_coefficients);
  ASSERT_EQ(left.normal.size(),right.normal.size());
  for(std::size_t i=0;i<left.normal.size();++i)for(unsigned k=0;k<4;++k){
    EXPECT_EQ(left.normal[i].neighbors[k],right.normal[i].neighbors[k]);
    EXPECT_EQ(left.normal[i].neighbor_edges[k],right.normal[i].neighbor_edges[k]);}
}
TEST(ActivityOperandsCuda, NativeUnselectedSupportLaterRemovalAndCompleteOrderedOutputs) {
  GpuFixture f;ASSERT_TRUE(f.Initialize());const auto initial=Read(f,0);
  const auto first=f.operands.Stage(0,f.Activity({1,1},{0,1}));ASSERT_TRUE(Good(first.report));
  const auto one=Read(f,first.staged_slot);Compare(f,initial,one,Mesh(f,{1,1},{0,1},1,1),first);
  EXPECT_DOUBLE_EQ(one.mains[0].coefficient,initial.mains[0].coefficient);
  // Discard leaves original slot byte-equivalent and hides only the alternate.
  f.operands.DiscardStaged(0,first.staged_slot);EXPECT_EQ(f.operands.view(1).main_count,0u);Exact(initial,Read(f,0));
  const auto retry=f.operands.Stage(0,f.Activity({1,1},{0,1},1,1,2));ASSERT_TRUE(Good(retry.report));Exact(one,Read(f,retry.staged_slot));
  unsigned accepted=retry.staged_slot;
  const auto unchanged=f.operands.Stage(accepted,f.Activity({0,1},{0,1},1,1,99));
  ASSERT_TRUE(Good(unchanged.report));EXPECT_FALSE(unchanged.changed);Exact(one,Read(f,unchanged.staged_slot));
  f.operands.DiscardStaged(accepted,accepted^1u);
  const auto second=f.operands.Stage(accepted,f.Activity({0,1},{0,0},1,1,100));ASSERT_TRUE(Good(second.report));
  const auto two=Read(f,second.staged_slot);Compare(f,one,two,Mesh(f,{0,1},{0,0},1,1),second);
  EXPECT_EQ(two.mains[0].coefficient,0);EXPECT_GT(second.removed_events,0u);EXPECT_TRUE(second.changed);
  accepted=second.staged_slot;
  const auto third=f.operands.Stage(accepted,f.Activity({0,0},{0,0},1,0,101));ASSERT_TRUE(Good(third.report));
  Compare(f,two,Read(f,third.staged_slot),Mesh(f,{0,0},{0,0},1,0),third);
}
TEST(ActivityOperandsCuda, FixedProfileCanStageBackIntoSlotZeroWithoutNormalStorage) {
  GpuFixture f(false);ASSERT_TRUE(f.Initialize());const auto zero=Read(f,0);
  auto first=f.operands.Stage(0,f.Activity({1,1},{0,1}));ASSERT_TRUE(Good(first.report));
  const auto one=Read(f,first.staged_slot);Compare(f,zero,one,Mesh(f,{1,1},{0,1},1,1),first);
  auto second=f.operands.Stage(first.staged_slot,f.Activity({0,1},{0,0},1,1,2));ASSERT_TRUE(Good(second.report));
  EXPECT_EQ(second.staged_slot,0u);EXPECT_EQ(second.free_count,0u);
  Compare(f,one,Read(f,0),Mesh(f,{0,1},{0,0},1,1),second);
  EXPECT_EQ(f.operands.view(0).normal_mains,nullptr);EXPECT_EQ(f.operands.view(0).free_mains,nullptr);
}
TEST(ActivityOperandsCuda, WallDeletionDisabledRetainsEveryOperand) {
  GpuFixture f(false,false);ASSERT_TRUE(f.Initialize());const auto old=Read(f,0);
  const auto report=f.operands.Stage(0,f.Activity({1,1},{0,0},1,0));ASSERT_TRUE(Good(report.report));
  EXPECT_FALSE(report.changed);EXPECT_EQ(report.affected_events,0u);EXPECT_EQ(report.orphan_secondaries,0u);
  const auto next=Read(f,report.staged_slot);Exact(old,next);Compare(f,old,next,Mesh(f,{1,1},{0,0},1,0),report);
}
TEST(ActivityOperandsCuda, NegativeMainCounterInitializationObeysFinalErosionDeclaration) {
  for(bool erosion:{false,true}) {
    SCOPED_TRACE(erosion);GpuFixture f(true,true,true,erosion);ASSERT_TRUE(f.Initialize());
    const auto old=Read(f,0);ASSERT_EQ(old.mains.size(),5u);ASSERT_LT(old.mains[2].coefficient,0);
    for(std::size_t i=0;i<old.connected.size();++i)EXPECT_EQ(old.connected[i],erosion&&i==2?2:0);
    const auto report=f.operands.Stage(0,f.Activity({1,1},{0,1}));ASSERT_TRUE(Good(report.report));
    const auto next=Read(f,report.staged_slot);
    Compare(f,old,next,Mesh(f,{1,1},{0,1},1,1),report);
    if(!erosion)for(auto count:next.connected)EXPECT_EQ(count,0);
  }
}
TEST(ActivityOperandsCuda, BadActivityAndAliasCannotAlterAcceptedOperands) {
  GpuFixture f;ASSERT_TRUE(f.Initialize());const auto before=Read(f,0);
  for(unsigned repeat=0;repeat<3;++repeat){
    auto bad=f.Activity({0,1},{1,1},1,1,repeat+1);
    EXPECT_EQ(f.operands.Stage(0,bad).report.status,n::TransactionStatus::ActivityChange);
    EXPECT_EQ(f.operands.view(1).main_count,0u);Exact(before,Read(f,0));
  }
  auto bad=f.Activity({1,1},{2,1});
  EXPECT_EQ(f.operands.Stage(0,bad).report.status,n::TransactionStatus::ActivityChange);Exact(before,Read(f,0));
  bad=f.Activity({1,1},{1,1});++bad.qeph.summary.count;
  EXPECT_EQ(f.operands.Stage(0,bad).report.status,n::TransactionStatus::SourceMismatch);Exact(before,Read(f,0));
  EXPECT_FALSE(f.operands.OutputDisjoint(f.operands.view(0).secondary_coefficients,sizeof(double)));
  f.operands.DiscardStaged(0,0);EXPECT_EQ(f.operands.view(0).main_count,before.mains.size());
  f.operands.DiscardStaged(0,1);Exact(before,Read(f,0));
}
TEST(ActivityOperandsCuda, ExactBudgetsDescriptorOnlyForecastAndBorrowedRangeChecks) {
  GpuFixture f;auto shape=f.Shape();
  auto forecast=a::State::Preflight(f.plan,f.source,&f.topology,f.units,shape);
  ASSERT_TRUE(Good(forecast.report));EXPECT_GT(forecast.owned_device_bytes,0u);
  EXPECT_EQ(forecast.source_plan_host_bytes,f.plan.forecast().output_bytes);
  a::Limits cap;cap.max_device_bytes=forecast.owned_device_bytes;cap.max_host_bytes=forecast.owned_host_bytes;
  cap.max_startup_host_bytes=forecast.startup_host_bytes;
  ASSERT_TRUE(Good(a::State::Preflight(f.plan,f.source,&f.topology,f.units,shape,cap).report));
  --cap.max_device_bytes;EXPECT_EQ(a::State::Preflight(f.plan,f.source,&f.topology,f.units,shape,cap).report.status,n::TransactionStatus::ResourceLimit);
  ++cap.max_device_bytes;--cap.max_startup_host_bytes;
  EXPECT_EQ(a::State::Preflight(f.plan,f.source,&f.topology,f.units,shape,cap).report.status,n::TransactionStatus::ResourceLimit);
  ASSERT_TRUE(f.Initialize());a::State wrong;
  auto aliased=f.borrowed;aliased.normal_coefficients=reinterpret_cast<double*>(aliased.mains);
  EXPECT_EQ(wrong.Initialize(f.plan,f.source,&f.topology,f.units,aliased,f.resources.stream).status,n::TransactionStatus::InvalidInput);
  EXPECT_EQ(wrong.allocations().owned_device_bytes,0u);
  shape=f.Shape();EXPECT_EQ(wrong.Initialize(f.plan,f.source,&f.topology,f.units,shape,f.resources.stream).status,n::TransactionStatus::InvalidInput);
}
} // namespace activity_operands_test
