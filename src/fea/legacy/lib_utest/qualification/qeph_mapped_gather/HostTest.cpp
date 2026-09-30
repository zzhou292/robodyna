// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <cstdio>

namespace qeph_gather_test {
TEST(QephMappedGather,FlatSourceOrderAndLateConnectivityRejection) {
  struct Element { std::size_t nodes[4]; };
  Element elements[]{{{3,0,2,1}},{{1,3,2,0}},{{4,2,1,3}}};
  std::array<std::uint32_t,7> offsets{};
  std::array<std::uint32_t,12> incidence{};
  ASSERT_TRUE(m::BuildIncidence(elements,3,6,offsets.data(),7,incidence.data(),12));
  EXPECT_EQ(offsets,(std::array<std::uint32_t,7>{0,2,5,8,11,12,12}));
  EXPECT_EQ(incidence,(std::array<std::uint32_t,12>{1,7,3,4,10,2,6,9,0,5,11,8}));
  const auto old_offsets=offsets;const auto old_incidence=incidence;
  elements[2].nodes[3]=6;
  EXPECT_FALSE(m::BuildIncidence(elements,3,6,offsets.data(),7,incidence.data(),12));
  EXPECT_EQ(offsets,old_offsets);EXPECT_EQ(incidence,old_incidence);
  elements[2].nodes[3]=4;
  EXPECT_FALSE(m::BuildIncidence(elements,3,6,offsets.data(),7,incidence.data(),12));
  // Count arithmetic rejects before reading poisoned borrowed arrays.
  EXPECT_FALSE(m::BuildIncidence(static_cast<Element*>(nullptr),UINT32_MAX,6,
      offsets.data(),7,incidence.data(),12));
}
TEST(QephMappedGather,ExactArenaCapRebaseLegacyAndFullVehicleForecast) {
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
  ASSERT_TRUE(full_old.Initialize(324094,372435,2ull<<30));
  ASSERT_TRUE(full.InitializeMapped(324094,372435,2ull<<30));
  std::printf("GATHER_FORECAST legacy_layout=%zu mapped_layout=%zu optional_tail=%zu header_pointer_bytes=%zu parents=324094 nodes=372435\n",
      full_old.bytes,full.bytes,full.bytes-full_old.bytes,sizeof(m::AssemblyMemory));
  EXPECT_LT(full.bytes-full_old.bytes,64u<<20);
}
TEST(QephMappedGather,AllParentMasksPreserveInitialValuesAndSerialSumBits) {
  Fixture fixture;
  for (unsigned epoch=0;epoch<2;++epoch) for (unsigned mask=0;mask<8;++mask) {
    fixture.Prepare(epoch);
    auto expected=fixture.input;
    const auto view=fixture.input.View(epoch);const auto cin=fixture.input.Cin();
    for (unsigned parent=0;parent<Parents;++parent) {
      auto& values=fixture.host->slab[0].element[parent];
      if (epoch && parent<3 && (mask&(1u<<parent))) {
        for (auto& force:values.internal_force) force={-0.,0.,-0.};
        for (auto& couple:values.internal_couple) couple={0.,-0.,0.};
        values.diagnostics.translational_stiffness=values.diagnostics.rotational_stiffness=0;
      }
      auto& prepared=fixture.host->assembly.parent[parent];
      prepared=m::PrepareAssemblyParent(fixture.host->model,values,parent,fixture.input.law[parent],view,epoch==0);
      ASSERT_EQ(prepared.status,q::BatchStatus::Success);
      if (fixture.input.law[parent]==fe::ShellSectionLaw::RigidSkin) continue;
      const auto& nodes=fixture.host->model.element[parent].nodes;
      ASSERT_EQ(fe::AccumulateNodalForces<4>(nodes,values.internal_force,values.internal_couple,
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
TEST(QephMappedGather,OverflowDoesNotPublishScratchAndRetryReplacesIt) {
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
TEST(QephMappedGather,ParentValidationKeepsLocalNodeAndResultPriority) {
  Fixture f;f.Prepare(1);auto view=f.input.View(1);
  auto& last=f.host->slab[0].element[2];last.internal_force[3].x=std::numeric_limits<double>::quiet_NaN();
  auto result=m::PrepareAssemblyParent(f.host->model,last,2,f.input.law[2],view,false);
  EXPECT_EQ(result.status,q::BatchStatus::NonfiniteResult);
  f.input.orientation[4*4]=0;
  result=m::PrepareAssemblyParent(f.host->model,last,2,f.input.law[2],view,false);
  EXPECT_EQ(result.status,q::BatchStatus::InvalidInput);EXPECT_EQ(result.node,4u);
  f.input.orientation[4*4]=1;last.internal_force[3].x=0;
  EXPECT_EQ(m::PrepareAssemblyParent(f.host->model,last,2,f.input.law[2],view,false).status,q::BatchStatus::Success);
}
} // namespace qeph_gather_test
