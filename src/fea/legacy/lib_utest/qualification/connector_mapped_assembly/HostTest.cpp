// SPDX-License-Identifier: MIT
#include "Fixture.h"
namespace connector_test {
TEST(ConnectorMappedAssemblyHost, SparseIncidenceKeepsSourceParentAndEndpointOrder) {
  Packet13 p;Prepare(p);
  EXPECT_EQ(p.storage.assembly.touched_count,4u);
  EXPECT_EQ((std::vector<std::uint32_t>(p.touched,p.touched+4)),(std::vector<std::uint32_t>{0,3,4,5}));
  EXPECT_EQ((std::vector<std::uint32_t>(p.incidence+p.offsets[0],p.incidence+p.offsets[1])),
      (std::vector<std::uint32_t>{0,3,4,9}));
  const auto before=p.offsets[0];p.elements[4].nodes[1]=p.elements[4].nodes[0];
  EXPECT_FALSE(fe::mapped_shell::BuildIncidence<2>(p.elements,Parents,Nodes,p.offsets,Nodes+1,p.incidence,2*Parents));
  EXPECT_EQ(p.offsets[0],before);
}
TEST(ConnectorMappedAssemblyHost, ScratchUsesEndpointBoundAndExactCapacityPreservesOutputOnFailure) {
  tl::util::BoundedArenaLayout arena(1u<<20);mc::Layout layout;
  ASSERT_TRUE(mc::AppendLayout(arena,Parents,Nodes,layout));
  EXPECT_EQ(layout.node.count,2*Parents);EXPECT_EQ(layout.touched_nodes.count,2*Parents);
  tl::util::BoundedArenaLayout exact(arena.bytes()),short_arena(arena.bytes()-1);mc::Layout exact_layout,short_layout;
  ASSERT_TRUE(mc::AppendLayout(exact,Parents,Nodes,exact_layout));
  EXPECT_FALSE(mc::AppendLayout(short_arena,Parents,Nodes,short_layout));EXPECT_EQ(short_layout.failure.count,0u);
  a::ArenaLayout legacy,mapped;
  ASSERT_TRUE(a::MakeLayout(1,Parents,{},legacy));
  ASSERT_TRUE(a::MakeLayout(1,Parents,{},mapped,Nodes));
  EXPECT_GT(mapped.bytes,legacy.bytes);EXPECT_EQ(legacy.assembly.failure.count,0u);
  b::ArenaLayout old25,new25;
  ASSERT_TRUE(b::MakeLayout(1,Parents,Nodes,1u<<20,old25));
  ASSERT_TRUE(b::MakeLayout(1,Parents,Nodes,1u<<20,new25,fe::type25::CapacityProfile::Legacy,true));
  EXPECT_GT(new25.bytes,old25.bytes);EXPECT_EQ(old25.assembly.failure.count,0u);
}
TEST(ConnectorMappedAssemblyHost, ConstructedSparseRosterAndAllEightDestinationRangesAreChecked) {
  tl::util::BoundedArenaLayout layout(1u<<20);mc::Layout regions;
  ASSERT_TRUE(mc::AppendLayout(layout,Parents,Nodes,regions));
  tl::util::HostArena arena;ASSERT_TRUE(arena.Initialize(layout.bytes()));
  ASSERT_TRUE(mc::Construct(arena,regions));auto memory=mc::Rebase(arena.data(),regions);
  Packet13 p;Prepare(p);ASSERT_TRUE(mc::Build(p.elements,Parents,Nodes,regions,memory));
  EXPECT_EQ(memory.touched_count,4u);
  for(unsigned i=0;i<4;++i) EXPECT_EQ(memory.touched_nodes[i],p.touched[i]);
  auto view=p.input.View();auto cin=p.input.Cin();
  EXPECT_TRUE(mc::ForceViewValid(view.forces,p.elements[0].nodes));
  view.forces.force_y=view.forces.force_x;
  EXPECT_FALSE(mc::ForceViewValid(view.forces,p.elements[0].nodes));
  EXPECT_FALSE(mc::StiffnessViewValid(cin.translational_stiffness,cin.translational_stiffness,Nodes,p.elements[0].nodes));
  EXPECT_FALSE(mc::StiffnessViewValid(cin.translational_stiffness,cin.rotational_stiffness,1,p.elements[0].nodes));
}
TEST(ConnectorMappedAssemblyHost, DistinctFailureOrderMatchesExistingFamilyContracts) {
  Packet13 p13;Packet25 p25;Prepare(p13);Prepare(p25);
  BadCoefficient(p13,0);BadCoefficient(p25,0);
  p13.input.orientation[4*4]=0;p25.input.orientation[4*4]=0;
  const auto x=StageHost<a::AssemblyFamily>(p13),y=StageHost<b::AssemblyFamily>(p25);
  EXPECT_EQ(mc::FailedParent(a::AssemblyFamily::Ordering,Parents,x),2u);
  EXPECT_EQ(mc::FailedParent(b::AssemblyFamily::Ordering,Parents,y),0u);
  EXPECT_EQ(p13.parent[2].node,4u);
}
TEST(ConnectorMappedAssemblyHost, ConstraintsAndZeroInverseCoefficientsRetainActualOwnerPolicy) {
  Packet13 p13;Packet25 p25;
  Prepare(p13);Prepare(p25);
  EXPECT_EQ(StageHost<a::AssemblyFamily>(p13),fe::mapped_shell::NoAssemblyFailure);
  EXPECT_EQ(StageHost<b::AssemblyFamily>(p25),fe::mapped_shell::NoAssemblyFailure);
  p13.input.fixed[3]=1;p25.input.fixed[3]=1;
  EXPECT_EQ(mc::FailedParent(a::AssemblyFamily::Ordering,Parents,StageHost<a::AssemblyFamily>(p13)),1u);
  EXPECT_EQ(mc::FailedParent(b::AssemblyFamily::Ordering,Parents,StageHost<b::AssemblyFamily>(p25)),1u);
}
} // namespace connector_test
