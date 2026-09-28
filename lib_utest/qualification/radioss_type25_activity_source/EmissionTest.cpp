// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../nodal_empty_cin/Fixture.h"
#include "lib_src/collision/radioss_type25/activity_source/Plan.h"
#include <gtest/gtest.h>
#include <algorithm>
namespace {
namespace n=tlfea::contact::radioss_type25;namespace a=n::activity_source;
using Status=n::TransactionStatus;
struct Fixture {
  nodal_empty_test::Fixture physical;
  std::array<n::lifecycle::Main,4> mains;
  std::array<std::uint64_t,2> selected{{100,200}};
  Fixture() {
    auto input=nodal_empty_test::SmallSource();input.nodes=4;
    auto& triangle=input.triangles[0];triangle.nodes={0,1,2};
    for(unsigned k=0;k<3;++k) {
      triangle.reference.node_ids[k]=input.quads[0].reference.node_ids[k];
      triangle.reference.position[k]=input.quads[0].reference.position[k];
    }
    auto coincident=input.quads[0];coincident.source_parent_id=101;input.quads.push_back(coincident);
    input.parents.push_back({tl::fea::ShellBindingFamily::Qeph,1,101,3,1,1});
    physical.Prepare(input,{7,7,7,7},{1,1,1,1});
    for(unsigned i=0;i<4;++i) {
      mains[i].global_id=i+1;mains[i].segment_type=i<2?int(i+3):-int(i-1);
    }
    for(unsigned k=0;k<4;++k) {
      mains[0].nodes[k]=physical.mapping.owner_index(physical.shells.qeph_nodes(0)[k]);
      mains[1].nodes[k]=physical.mapping.owner_index(physical.shells.t3_nodes(0)[k<3?k:2]);
    }
    constexpr unsigned q[]{1,0,3,2},t[]{1,0,2,2};
    for(unsigned k=0;k<4;++k){mains[2].nodes[k]=mains[0].nodes[q[k]];mains[3].nodes[k]=mains[1].nodes[t[k]];}
  }
  n::ContactSourceInput Source() const {
    n::ContactSourceInput source;source.source_id=1;source.topology_generation=1;
    source.selection.mains=mains.data();source.selection.main_count=4;source.selection.node_count=4;
    source.selection.generation=7;source.primary_main_count=2;source.primary_parent_ids=selected.data();return source;
  }
};
a::Controls Controls(){return {a::Deletion::ContainingElement,false,n::startup::SolidErosion::Disabled};}
std::vector<std::uint32_t> Emitted(a::View view,std::uint64_t id) {
  for(std::size_t p=0;p<view.parents.size();++p)if(view.parents[p].source_element_id==id) {
    return {view.emitting_mains.data()+view.emitting_offsets[p],view.emitting_mains.data()+view.emitting_offsets[p+1]};
  }
  return {};
}
TEST(NativeActivityEmission, ReverseContainmentDistinguishesTriangleAndQuadDeletion) {
  Fixture f;a::Plan plan;const auto source=f.Source();
  const auto report=plan.Initialize({f.physical.physical,nullptr},source,Controls());
  ASSERT_EQ(report.status,Status::Ok)<<report.message;const auto view=plan.view();
  EXPECT_EQ(Emitted(view,100),(std::vector<std::uint32_t>{1,3}));
  EXPECT_EQ(Emitted(view,200),(std::vector<std::uint32_t>{1,2,3,4}));
  // Q4 contains the T3 main and can preserve its support, but its four-node
  // deletion face does not emit that smaller registered T3 main.
  std::vector<std::uint64_t> support;
  for(auto i=view.containing_offsets[1];i<view.containing_offsets[2];++i)
    support.push_back(view.parents[view.containing_parents[i]].source_element_id);
  EXPECT_NE(std::find(support.begin(),support.end(),100),support.end());
}
TEST(NativeActivityEmission, UnselectedCoincidentOwnersKeepSeparateOrderedExpandedEvents) {
  Fixture f;a::Plan plan;const auto source=f.Source();
  ASSERT_EQ(plan.Initialize({f.physical.physical,nullptr},source,Controls()).status,Status::Ok);
  const auto view=plan.view();
  EXPECT_EQ(Emitted(view,101),(std::vector<std::uint32_t>{1,3}));
  EXPECT_EQ(view.emitting_mains.size(),8u);EXPECT_EQ(view.emitting_offsets.size(),view.parents.size()+1);
  for(std::size_t p=0;p<view.parents.size();++p)
    for(auto i=view.emitting_offsets[p]+1;i<view.emitting_offsets[p+1];++i)
      EXPECT_LT(view.emitting_mains[i-1],view.emitting_mains[i]);
  // The repeated fourth triangle corner cannot duplicate a registered event.
  const auto triangle=Emitted(view,200);EXPECT_EQ(std::count(triangle.begin(),triangle.end(),2u),1);
  EXPECT_EQ(std::count(triangle.begin(),triangle.end(),4u),1);
}
TEST(NativeActivityEmission, CompleteEmissionReservationRejectsWithoutPartialPlanThenRetries) {
  Fixture f;const auto source=f.Source();
  const auto forecast=a::Plan::Preflight({f.physical.physical,nullptr},source,Controls());
  ASSERT_EQ(forecast.report.status,Status::Ok);ASSERT_GT(forecast.counts.emitting_capacity,0u);
  a::Limits limits;limits.emitting_mains=forecast.counts.emitting_capacity-1;a::Plan plan;
  EXPECT_EQ(plan.Initialize({f.physical.physical,nullptr},source,Controls(),limits).status,Status::ResourceLimit);
  EXPECT_FALSE(plan.initialized());limits.emitting_mains=forecast.counts.emitting_capacity;
  EXPECT_EQ(plan.Initialize({f.physical.physical,nullptr},source,Controls(),limits).status,Status::Ok);
}
}
