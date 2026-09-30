#include "lib_src/constraints/NodalRigidPartTopology.h"
#include <algorithm>
#include <array>
#include <gtest/gtest.h>
#include <vector>

namespace {
namespace r=tl::fea::rigid;using S=r::PartTopologyStatus;
struct Fixture {
  std::array<r::SourceNodeId,6> a{91,7,18,22,17,42};
  std::array<r::SourceNodeId,2> x{111,101};
  std::array<r::SourceNodeId,8> expected{7,17,18,22,42,91,101,111};
  std::array<r::SourceNodeId,2> other{500,400};
  std::array<r::PartTopologyPartInput,3> parts{{{30,a.data(),2},{10,a.data()+2,2},{20,a.data()+4,2}}};
  std::array<r::PartTopologyExtraInput,2> extra{{{10,81,x.data(),1},{30,82,x.data()+1,1}}};
  r::PartTopologyMerge merge{30,20};
  r::PartTopologyInput Input() const {
    return {917,parts.data(),parts.size(),extra.data(),extra.size(),&merge,1,
      expected.data(),expected.size(),other.data(),other.size(),{}};
  }
};
TEST(RigidPartTopology,OwnsSourceOrderAndSeparatesOriginalPartsFromMergedRoots) {
  r::NodalRigidPartTopology model;
  {
    Fixture f;ASSERT_TRUE(model.Initialize(f.Input()));
    std::fill(f.a.begin(),f.a.end(),999);std::fill(f.x.begin(),f.x.end(),999);
  }
  EXPECT_EQ(model.source_instance_id(),917u);EXPECT_EQ(model.part_count(),3u);
  EXPECT_EQ(model.extra_count(),2u);EXPECT_EQ(model.merge_count(),1u);
  EXPECT_EQ(model.root_count(),2u);EXPECT_EQ(model.member_count(),8u);
  const std::vector<r::SourceNodeId> expected{91,7,101,17,42,18,22,111};
  EXPECT_EQ(std::vector<r::SourceNodeId>(model.root_members(),model.root_members()+8),expected);
  EXPECT_EQ(model.parts()[0].source_part_id,30u);EXPECT_EQ(model.parts()[2].root_index,0u);
  EXPECT_EQ(model.extras()[0].source_node_set_id,81u);EXPECT_EQ(model.extras()[0].part_index,1u);
  EXPECT_EQ(model.roots()[0].original_primary_count,2u);EXPECT_EQ(model.roots()[1].original_primary_count,1u);
  EXPECT_EQ(model.roots()[0].member_count,5u);EXPECT_EQ(model.other_rigid_members()[0],500u);
  Fixture retry;EXPECT_EQ(model.Initialize(retry.Input()).status,S::AlreadyInitialized);
  EXPECT_EQ(model.root_members()[0],91u);
}
TEST(RigidPartTopology,MissingLateMembersAndGroupOverlapRejectBeforePublicationThenRetry) {
  for(unsigned fault=0;fault<5;++fault) {
    Fixture f;r::NodalRigidPartTopology model;
    if(fault==0)f.expected.back()=999;
    if(fault==1)f.other.back()=111;
    if(fault==2)f.x.back()=91;
    if(fault==3)f.parts.back().source_part_id=30;
    if(fault==4)f.extra.back().source_node_set_id=81;
    EXPECT_FALSE(model.Initialize(f.Input()))<<fault;
    EXPECT_FALSE(model.prepared());EXPECT_EQ(model.parts(),nullptr);EXPECT_EQ(model.root_count(),0u);
    Fixture clean;ASSERT_TRUE(model.Initialize(clean.Input()))<<fault;
    EXPECT_EQ(model.root_members()[7],111u);
  }
}
TEST(RigidPartTopology,UnsupportedMergeNetworksAndCountByteCapsAreExplicit) {
  Fixture f;
  for(const r::PartTopologyMerge bad:std::array<r::PartTopologyMerge,3>{{{30,30},{999,20},{20,999}}}) {
    f.merge=bad;r::NodalRigidPartTopology model;
    EXPECT_EQ(model.Initialize(f.Input()).status,S::UnsupportedMerge);
  }
  f.merge={30,20};auto in=f.Input();in.part_count=SIZE_MAX;
  r::NodalRigidPartTopology empty;
  EXPECT_EQ(empty.Initialize(in).status,S::ResourceLimit);
  in=f.Input();in.parts=reinterpret_cast<const r::PartTopologyPartInput*>(UINTPTR_MAX-4);
  EXPECT_EQ(empty.Initialize(in).status,S::InvalidInput);
  in=f.Input();in.limits.max_host_bytes=1;
  EXPECT_EQ(empty.Initialize(in).status,S::ResourceLimit);EXPECT_FALSE(empty.prepared());
  r::NodalRigidPartTopology measured;ASSERT_TRUE(measured.Initialize(f.Input()));
  in=f.Input();in.limits.max_host_bytes=measured.startup_payload_bytes()-1;
  EXPECT_EQ(empty.Initialize(in).status,S::ResourceLimit);
  in.limits.max_host_bytes++;ASSERT_TRUE(empty.Initialize(in));
}
TEST(RigidPartTopology,ChainsAndMultipleChildrenFailInEitherDeclaredMergeOrder) {
  const std::array<r::SourceNodeId,8> nodes{1,2,3,4,5,6,7,8};
  const std::array<r::PartTopologyPartInput,4> parts{{{1,nodes.data(),2},{2,nodes.data()+2,2},
    {3,nodes.data()+4,2},{4,nodes.data()+6,2}}};
  const r::PartTopologyMerge cases[3][2]={{{1,2},{2,3}},{{1,2},{1,3}},{{1,2},{2,1}}};
  for(const auto& scenario:cases) {
    std::array<r::PartTopologyMerge,2> merges{scenario[0],scenario[1]};
    for(unsigned reverse=0;reverse<2;++reverse) {
      r::PartTopologyInput in{1,parts.data(),4,nullptr,0,merges.data(),2,nodes.data(),8};
      r::NodalRigidPartTopology model;EXPECT_EQ(model.Initialize(in).status,S::UnsupportedMerge);
      EXPECT_FALSE(model.prepared());std::swap(merges[0],merges[1]);
    }
  }
}
} // namespace
