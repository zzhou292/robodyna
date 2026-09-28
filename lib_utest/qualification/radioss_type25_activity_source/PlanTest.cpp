// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../radioss_type25_runtime/PhysicalMainFixture.h"
#include "lib_src/collision/radioss_type25/activity_source/Plan.h"
#include <set>
namespace {
namespace a=tlfea::contact::radioss_type25::activity_source;
namespace s=tlfea::contact::radioss_type25::startup;
using Status=tlfea::contact::radioss_type25::TransactionStatus;
a::Controls MixedControls(){return {a::Deletion::AllSupports,false,s::SolidErosion::Enabled};}
a::Controls OrdinaryControls(){return {a::Deletion::AllSupports,false,s::SolidErosion::Disabled};}
std::set<std::uint64_t> Containing(a::View v,std::size_t primary) {
  std::set<std::uint64_t> result;
  for(auto i=v.containing_offsets[primary];i<v.containing_offsets[primary+1];++i)
    result.insert(v.parents[v.containing_parents[i]].source_element_id);
  return result;
}
TEST(NativeActivitySource, CompleteGlobalRosterRetainsUnselectedCoincidentShellSupport) {
  type25_source_test::FullLedgerFixture f;auto source=f.Contact();a::Plan plan;
  const auto result=plan.Initialize(f.physical,source,OrdinaryControls());
  ASSERT_EQ(result.status,Status::Ok)<<result.message;
  const auto v=plan.view();EXPECT_TRUE(plan.Matches(f.physical));
  const auto containing=Containing(v,0);EXPECT_TRUE(containing.count(100));EXPECT_TRUE(containing.count(101));
  EXPECT_EQ(v.main_to_primary[0],v.main_to_primary[source.primary_main_count]);
  EXPECT_EQ(v.mains[0].first,v.mains[source.primary_main_count].first);
  EXPECT_EQ(v.origins.size(),source.primary_main_count);
}
TEST(NativeActivitySource, MixedFinalSupportsRemainDistinctFromCompleteRawOriginsAndContainment) {
  type25_physical_main_test::Fixture f;a::Plan plan;
  const auto result=plan.Initialize(f.physical.physical,f.sides,f.post,MixedControls());
  ASSERT_EQ(result.status,Status::Ok)<<result.message;const auto v=plan.view();
  EXPECT_EQ(v.parents[v.mains[2].first].source_element_id,9101u);
  EXPECT_EQ(v.parents[v.mains[2].second].source_element_id,9100u);
  ASSERT_EQ(v.origins.size(),4u);EXPECT_EQ(v.parents[v.origins[2].parent].source_element_id,9100u);
  EXPECT_EQ(v.parents[v.origins[3].parent].source_element_id,9101u);
  EXPECT_TRUE(Containing(v,2).count(9100));EXPECT_TRUE(Containing(v,2).count(9101));
  const auto q=Containing(v,0);EXPECT_TRUE(q.count(100));EXPECT_TRUE(q.count(101));EXPECT_TRUE(q.count(103));
}
TEST(NativeActivitySource, NodeIncidenceHasOneEntryPerActualParentAndNoBareMassSupport) {
  type25_physical_main_test::Fixture f;a::Plan plan;
  ASSERT_EQ(plan.Initialize(f.physical.physical,f.sides,f.post,MixedControls()).status,Status::Ok);
  const auto v=plan.view();
  for(std::size_t node=0;node+1<v.node_offsets.size();++node) {
    std::uint32_t prior=0;bool first=true;
    for(auto i=v.node_offsets[node];i<v.node_offsets[node+1];++i) {
      EXPECT_LT(v.node_parents[i],v.parents.size());
      if(!first)EXPECT_GT(v.node_parents[i],prior);prior=v.node_parents[i];first=false;
    }
  }
  const auto mass_node=f.physical.domain.Find(777);ASSERT_NE(mass_node,SIZE_MAX);
  EXPECT_EQ(v.node_offsets[mass_node],v.node_offsets[mass_node+1]);
  for(const auto& parent:v.parents)EXPECT_NE(parent.source_element_id,18000u);
}
TEST(NativeActivitySource, TypedFamiliesPreserveActualFamilyLocalIndices) {
  type25_physical_main_test::Fixture f;a::Plan plan;
  ASSERT_EQ(plan.Initialize(f.physical.physical,f.sides,f.post,MixedControls()).status,Status::Ok);
  std::array<std::size_t,a::FamilyCount> observed{};
  for(const auto& p:plan.view().parents) {
    const auto family=static_cast<std::size_t>(p.family);ASSERT_LT(family,a::FamilyCount);
    EXPECT_EQ(p.family_index,observed[family]++);
  }
  EXPECT_EQ(observed,plan.forecast().counts.families);
  EXPECT_EQ(observed[static_cast<std::size_t>(a::Family::Qeph)],2u);
  EXPECT_EQ(observed[static_cast<std::size_t>(a::Family::T3)],1u);
  EXPECT_EQ(observed[static_cast<std::size_t>(a::Family::Qbat)],1u);
  EXPECT_GT(observed[static_cast<std::size_t>(a::Family::Type13)],0u);
  EXPECT_GT(observed[static_cast<std::size_t>(a::Family::Type25)],0u);
}
TEST(NativeActivitySource, WrongControlsAndLateOriginFailureLeavePlanEmptyThenRetry) {
  type25_physical_main_test::Fixture f;a::Plan plan;auto controls=MixedControls();
  controls.solid_erosion=s::SolidErosion::Disabled;
  EXPECT_EQ(plan.Initialize(f.physical.physical,f.sides,f.post,controls).status,Status::SourceMismatch);
  EXPECT_FALSE(plan.initialized());auto bad=f.sides;auto origins=f.origins;origins.back().physical_parent_id=999999;
  bad.raw_origins=origins.data();
  EXPECT_EQ(plan.Initialize(f.physical.physical,bad,f.post,MixedControls()).status,Status::SourceMismatch);
  EXPECT_FALSE(plan.initialized());
  EXPECT_EQ(plan.Initialize(f.physical.physical,f.sides,f.post,MixedControls()).status,Status::Ok);
  EXPECT_EQ(plan.Initialize(f.physical.physical,f.sides,f.post,MixedControls()).status,Status::AlreadyInitialized);
}
TEST(NativeActivitySource, ExactRetainedAndPeakResourceBoundariesAreHonored) {
  type25_physical_main_test::Fixture f;
  const auto forecast=a::Plan::Preflight(f.physical.physical,f.sides,f.post,MixedControls());
  ASSERT_EQ(forecast.report.status,Status::Ok)<<forecast.report.message;
  a::Limits limits;limits.output_bytes=forecast.output_bytes;limits.startup_bytes=forecast.startup_bytes;
  EXPECT_EQ(a::Plan::Preflight(f.physical.physical,f.sides,f.post,MixedControls(),limits).report.status,Status::Ok);
  --limits.output_bytes;
  EXPECT_EQ(a::Plan::Preflight(f.physical.physical,f.sides,f.post,MixedControls(),limits).report.status,Status::ResourceLimit);
  ++limits.output_bytes;--limits.startup_bytes;
  EXPECT_EQ(a::Plan::Preflight(f.physical.physical,f.sides,f.post,MixedControls(),limits).report.status,Status::ResourceLimit);
  ++limits.startup_bytes;limits.containing_parents=forecast.counts.containing_capacity-1;
  EXPECT_EQ(a::Plan::Preflight(f.physical.physical,f.sides,f.post,MixedControls(),limits).report.status,Status::ResourceLimit);
}
TEST(NativeActivitySource, MalformedOrdinaryOppositeAndUnknownControlAreRejected) {
  type25_source_test::FullLedgerFixture f;auto source=f.Contact();a::Plan plan;
  auto controls=OrdinaryControls();controls.deletion=static_cast<a::Deletion>(255);
  EXPECT_EQ(plan.Initialize(f.physical,source,controls).status,Status::UnsupportedProfile);
  auto mains=f.mains;std::swap(mains[source.primary_main_count].nodes[0],mains[source.primary_main_count].nodes[1]);
  source.selection.mains=mains.data();
  EXPECT_EQ(plan.Initialize(f.physical,source,OrdinaryControls()).status,Status::SourceMismatch);
  EXPECT_FALSE(plan.initialized());
}
}
