#include "topology_fixture/YarisRigidTopologyFixture.h"
#include <algorithm>
#include <gtest/gtest.h>
#include <vector>

TEST(RigidPartTopology,OriginalYarisTwentyTwoBodiesProduceTwentyDisjointRoots) {
  namespace f=yaris_rigid_topology_fixture;
  namespace r=tl::fea::rigid;
  const r::PartTopologyInput in{0x67208317e6c8eb1dULL,
    f::Parts,std::size(f::Parts),f::Extras,std::size(f::Extras),f::Merges,std::size(f::Merges),
    f::Expected,std::size(f::Expected),f::Other,std::size(f::Other),{}};
  r::NodalRigidPartTopology model;ASSERT_TRUE(model.Initialize(in));
  EXPECT_EQ(model.part_count(),22u);EXPECT_EQ(model.extra_count(),20u);
  EXPECT_EQ(model.root_count(),20u);EXPECT_EQ(model.member_count(),5452u);
  EXPECT_EQ(model.other_rigid_member_count(),7539u);
  std::size_t primaries=0,extras=0;
  for(std::size_t i=0;i<model.root_count();++i) {
    const auto& root=model.roots()[i];
    EXPECT_EQ(model.parts()[root.part_index].source_part_id,f::RootParts[i]);
    EXPECT_EQ(root.member_count,f::RootCounts[i]);primaries+=root.original_primary_count;
  }
  for(std::size_t i=0;i<model.extra_count();++i)extras+=model.extras()[i].member_count;
  EXPECT_EQ(primaries,22u);EXPECT_EQ(extras,302u);
  std::vector<r::SourceNodeId> sorted(model.root_members(),model.root_members()+model.member_count());
  std::sort(sorted.begin(),sorted.end());
  EXPECT_TRUE(std::equal(sorted.begin(),sorted.end(),std::begin(f::Expected)));
  EXPECT_TRUE(std::equal(model.original_members(),model.original_members()+model.member_count(),std::begin(f::Members)));
  // This is the complete source ID topology, including the 54 auxiliary nodes.
  // Neither positive mass, missing native IN, nor live constrained dynamics is
  // admitted by successful topology construction.
  EXPECT_LT(model.startup_payload_bytes(),1024u*1024u);
}
