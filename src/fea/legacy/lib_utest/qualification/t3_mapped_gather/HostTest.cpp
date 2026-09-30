// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cstdio>

namespace t3_gather_test {
static_assert(sizeof(m::AssemblyParent)==56 && sizeof(m::AssemblyNode)==72);
static_assert(sizeof(m::AssemblyMemory)==48 && sizeof(m::AssemblyLayout)==152);
TEST(T3MappedGather,FlatSourceOrderAndLateConnectivityRejection) {
  struct Element { std::size_t nodes[3]; };
  Element elements[]{{{3,0,2}},{{1,3,2}},{{4,2,1}}};
  std::array<std::uint32_t,7> offsets{};
  std::array<std::uint32_t,9> incidence{};
  ASSERT_TRUE(m::BuildIncidence(elements,3,6,offsets.data(),7,incidence.data(),9));
  EXPECT_EQ(offsets,(std::array<std::uint32_t,7>{0,1,3,6,8,9,9}));
  EXPECT_EQ(incidence,(std::array<std::uint32_t,9>{1,3,8,2,5,7,0,4,6}));
  const auto old_offsets=offsets;const auto old_incidence=incidence;
  elements[2].nodes[2]=6;
  EXPECT_FALSE(m::BuildIncidence(elements,3,6,offsets.data(),7,incidence.data(),9));
  EXPECT_EQ(offsets,old_offsets);EXPECT_EQ(incidence,old_incidence);
  elements[2].nodes[2]=4;
  EXPECT_FALSE(m::BuildIncidence(elements,3,6,offsets.data(),7,incidence.data(),9));
  // Count arithmetic rejects before reading poisoned borrowed arrays.
  EXPECT_FALSE(m::BuildIncidence(static_cast<Element*>(nullptr),UINT32_MAX,6,
      offsets.data(),7,incidence.data(),9));
}
TEST(T3MappedGather,ExactArenaCapRebaseLegacyAndFullVehicleForecast) {
  b::Layout legacy,gather;
  ASSERT_TRUE(legacy.Initialize(Parents,Nodes,1u<<20));
  ASSERT_TRUE(gather.InitializeMapped(Parents,Nodes,1u<<20));
  EXPECT_GT(gather.bytes,legacy.bytes);EXPECT_EQ(legacy.assembly.bytes,0u);
  const auto bytes=gather.bytes;
  ASSERT_TRUE(gather.InitializeMapped(Parents,Nodes,bytes));
  EXPECT_FALSE(gather.InitializeMapped(Parents,Nodes,bytes-1));EXPECT_EQ(gather.bytes,bytes);
  tl::util::HostArena arena;ASSERT_TRUE(arena.Initialize(bytes));
  const auto* host=gather.Construct(arena);ASSERT_NE(host,nullptr);
  const auto copied=gather.Rebase(*host,arena.data());
  EXPECT_EQ(copied.assembly.node,host->assembly.node);EXPECT_EQ(copied.assembly.failure,host->assembly.failure);
  ASSERT_TRUE(gather.Initialize(Parents,Nodes,bytes));EXPECT_EQ(gather.assembly.bytes,0u);
  b::Layout full_old,full;
  ASSERT_TRUE(full_old.Initialize(21301,372435,2ull<<30));
  ASSERT_TRUE(full.InitializeMapped(21301,372435,2ull<<30));
  std::printf("GATHER_FORECAST legacy_layout=%zu mapped_layout=%zu optional_tail=%zu header_pointer_bytes=%zu parents=21301 nodes=372435\n",
      full_old.bytes,full.bytes,full.bytes-full_old.bytes,sizeof(m::AssemblyMemory));
  EXPECT_EQ(full.bytes-full_old.bytes,29753540u+32768u);
  EXPECT_EQ(sizeof(m::AssemblyMemory),48u);
  EXPECT_EQ(sizeof(m::AssemblyLayout),152u);
  // Only fixed metadata grows for the old initializer. Every capacity-sized
  // region remains exclusive to InitializeMapped and its host/device forecast.
  struct PreviousHeader {
    b::Model model;
    b::Slab slab[2];
    b::Control control;
    q::Status* candidate_status;
  };
  EXPECT_EQ(sizeof(b::Storage)-sizeof(PreviousHeader),48u);
  RecordProperty("storage_header_bytes",std::to_string(sizeof(b::Storage)));
  RecordProperty("layout_header_bytes",std::to_string(sizeof(b::Layout)));
  RecordProperty("full_mapped_arena_bytes",std::to_string(full.bytes));
  RecordProperty("full_optional_tail_bytes",std::to_string(full.bytes-full_old.bytes));
}
TEST(T3MappedGather,AllParentMasksPreserveInitialValuesAndSerialSumBits) {
  Fixture fixture;
  for (unsigned epoch=0;epoch<2;++epoch) for (unsigned mask=0;mask<8;++mask) {
    fixture.Prepare(epoch);
    auto expected=fixture.input;
    const auto view=fixture.input.View(epoch);const auto cin=fixture.input.Cin();
    for (unsigned parent=0;parent<Parents;++parent) {
      auto& values=fixture.host->slab[0].element[parent];
      if (epoch && parent<3 && (mask&(1u<<parent))) {
        Remove(fixture.host->model.element[parent].reference,values);
      }
      auto& prepared=fixture.host->assembly.parent[parent];
      prepared=m::PrepareAssemblyParent(fixture.host->model,values,parent,fixture.input.law[parent],view,epoch==0);
      ASSERT_EQ(prepared.status,q::BatchStatus::Success);
      if (fixture.input.law[parent]==fe::ShellSectionLaw::RigidSkin) continue;
      const auto& nodes=fixture.host->model.element[parent].nodes;
      ASSERT_EQ(fe::AccumulateNodalForces<3>(nodes,values.internal_force,values.internal_couple,
          expected.View(epoch).forces,-1),fe::NodalForceAssemblyStatus::Success);
      ASSERT_TRUE(fe::shell_nodal_stiffness::Add(nodes,prepared.stiffness,expected.values[6],expected.values[7],Nodes));
    }
    for (unsigned node=0;node<Nodes;++node) {
      ASSERT_EQ(m::GatherAssemblyNode(node,fixture.host->assembly,fixture.host->slab[0].element,
          fixture.input.law,view.forces,cin.translational_stiffness,cin.rotational_stiffness,
          fixture.host->assembly.node[node]),UINT32_MAX);
    }
    for (unsigned node=0;node<Nodes;++node) m::PublishAssemblyNode(node,fixture.host->assembly.node[node],
        view.forces,cin.translational_stiffness,cin.rotational_stiffness);
    Compare(fixture.input,expected);
    if (epoch==1 && mask==0) {
      EXPECT_EQ(fixture.input.values[0][0],1.);
      const double reordered=(0x1p54+1.)-0x1p54;
      EXPECT_NE(fixture.input.values[0][0],reordered);
    }
  }
}
TEST(T3MappedGather,OverflowDoesNotPublishScratchAndRetryReplacesIt) {
  Fixture f;f.Prepare(1);auto view=f.input.View(1);auto cin=f.input.Cin();
  for (unsigned parent=0;parent<Parents;++parent) f.host->assembly.parent[parent]=m::PrepareAssemblyParent(
      f.host->model,f.host->slab[0].element[parent],parent,f.input.law[parent],view,false);
  f.input.values[7][4]=std::numeric_limits<double>::max();
  f.host->assembly.parent[2].stiffness.rotation[1]=std::numeric_limits<double>::max();
  const auto before=f.input;
  m::AssemblyNode output;output.value[0]=12345;
  EXPECT_EQ(m::GatherAssemblyNode(4,f.host->assembly,f.host->slab[0].element,f.input.law,
      view.forces,cin.translational_stiffness,cin.rotational_stiffness,output),2u);
  EXPECT_EQ(output.value[0],12345);Compare(f.input,before);
  f.host->assembly.parent[2].stiffness.rotation[1]=0;
  ASSERT_EQ(m::GatherAssemblyNode(4,f.host->assembly,f.host->slab[0].element,f.input.law,
      view.forces,cin.translational_stiffness,cin.rotational_stiffness,output),UINT32_MAX);
  EXPECT_TRUE(output.touched);EXPECT_EQ(output.value[7],std::numeric_limits<double>::max());
}
TEST(T3MappedGather,ParentValidationKeepsLocalNodeAndResultPriority) {
  Fixture f;f.Prepare(1);auto view=f.input.View(1);
  auto& last=f.host->slab[0].element[2];last.internal_force[2].x=std::numeric_limits<double>::quiet_NaN();
  auto result=m::PrepareAssemblyParent(f.host->model,last,2,f.input.law[2],view,false);
  EXPECT_EQ(result.status,q::BatchStatus::NonfiniteResult);
  f.input.orientation[4*4]=0;
  result=m::PrepareAssemblyParent(f.host->model,last,2,f.input.law[2],view,false);
  EXPECT_EQ(result.status,q::BatchStatus::InvalidInput);EXPECT_EQ(result.node,4u);
  f.input.orientation[4*4]=1;last.internal_force[2].x=0;
  EXPECT_EQ(m::PrepareAssemblyParent(f.host->model,last,2,f.input.law[2],view,false).status,q::BatchStatus::Success);
}
} // namespace t3_gather_test
