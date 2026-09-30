// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include <vector>
#include "SerialMeasure.h"
namespace qbat_gather_test {
TEST(QbatMappedGatherHost,ExactOrderedEightChannelScatterKeepsIncomingValuesAndSignedZeros) {
  Fixture f;ASSERT_FALSE(HasFailure());
  for(unsigned epoch=0;epoch<3;++epoch) for(unsigned mask=0;mask<(epoch?16u:1u);++mask) {
    f.Prepare(epoch);
    for(unsigned parent=0;parent<Parents;++parent) if(mask&(1u<<parent))
      Remove(f.host->slab[0].element[parent]);
    auto serial=f.input;
    const auto view=f.input.View(epoch);const auto cin=f.input.Cin();
    for(unsigned parent=0;parent<Parents;++parent) {
      const auto& element=f.host->model.element[parent];const auto& result=f.host->slab[0].element[parent];
      auto& prepared=f.host->assembly.parent[parent];
      prepared=m::PrepareAssemblyParent(f.host->model,result,parent,view,epoch==0);
      ASSERT_EQ(prepared.status,q::BatchStatus::Success);
      ASSERT_EQ(fe::AccumulateNodalForces<4>(element.nodes,result.internal_force_n,result.internal_couple_nm,
          serial.View(epoch).forces,-1),fe::NodalForceAssemblyStatus::Success);
      ASSERT_TRUE(m::AddStiffness(element.nodes,prepared.stiffness,serial.values[6],serial.values[7],Nodes));
    }
    for(unsigned node=0;node<Nodes;++node) {
      m::AssemblyNode value;
      ASSERT_EQ(m::GatherAssemblyNode(node,f.host->assembly,f.host->slab[0].element,view.forces,
          cin.translational_stiffness,cin.rotational_stiffness,value),UINT32_MAX);
      fe::mapped_shell::PublishNode(node,value,view.forces,cin.translational_stiffness,cin.rotational_stiffness);
    }
    Compare(f.input,serial);
  }
}
TEST(QbatMappedGatherHost,IncidenceValidatesLateInputBeforeWritingAndRetainsSlotOrder) {
  Fixture f;ASSERT_FALSE(HasFailure());
  const auto offsets=std::vector<std::uint32_t>(f.host->assembly.offsets,f.host->assembly.offsets+Nodes+1);
  const auto incidence=std::vector<std::uint32_t>(f.host->assembly.incidence,f.host->assembly.incidence+4*Parents);
  std::vector<std::uint32_t> expected;
  for(unsigned p=0;p<Parents;++p) for(unsigned slot=0;slot<4;++slot)
    if(f.host->model.element[p].nodes[slot]==0) expected.push_back(4*p+slot);
  EXPECT_EQ(std::vector<std::uint32_t>(incidence.begin()+offsets[0],incidence.begin()+offsets[1]),expected);
  f.host->model.element[Parents-1].nodes[3]=Nodes;
  EXPECT_FALSE(fe::mapped_shell::BuildIncidence<4>(f.host->model.element,Parents,Nodes,
      f.host->assembly.offsets,Nodes+1,f.host->assembly.incidence,4*Parents));
  EXPECT_EQ(std::vector<std::uint32_t>(f.host->assembly.offsets,f.host->assembly.offsets+Nodes+1),offsets);
  EXPECT_EQ(std::vector<std::uint32_t>(f.host->assembly.incidence,f.host->assembly.incidence+4*Parents),incidence);
}
TEST(QbatMappedGatherHost,CompleteMappedTailUsesInclusiveCapsWithoutAllocatingFullSource) {
  b::Layout old,mapped;
  ASSERT_TRUE(old.Initialize(4250,376930,0,fe::MaxVehicleShellResidentDeviceBytes));
  ASSERT_TRUE(mapped.InitializeMapped(4250,376930,0,fe::MaxVehicleShellResidentDeviceBytes));
  EXPECT_GT(mapped.bytes,old.bytes);
  const auto exact=mapped.bytes;const auto saved=mapped.bytes;
  EXPECT_FALSE(mapped.InitializeMapped(4250,376930,0,exact-1));EXPECT_EQ(mapped.bytes,saved);
  EXPECT_TRUE(mapped.InitializeMapped(4250,376930,0,exact));
  EXPECT_FALSE(mapped.InitializeMapped(SIZE_MAX,4,0,SIZE_MAX));
  EXPECT_FALSE(mapped.InitializeMapped(1,SIZE_MAX,0,SIZE_MAX));
  EXPECT_EQ(old.assembly.bytes,0u);
  RecordProperty("original_v5_mapped_bytes",std::to_string(exact));
  RecordProperty("mapped_tail_bytes",std::to_string(exact-old.bytes));
}
TEST(QbatMappedGatherHost,CachedValidationAndMaximumKeepFrozenSignedDiagnosticOrderAndFailurePrefixes) {
  Fixture f;ASSERT_FALSE(HasFailure());f.Prepare(2,false);
  for(unsigned fault=0;fault<4;++fault) {
    f.Prepare(2,false);auto view=f.input.Prepared(2);auto expected=Identity(2),actual=expected;
    if(fault==1) f.host->slab[1].element[3].point[3].force_volume_m3=std::numeric_limits<double>::quiet_NaN();
    if(fault==2) f.input.endpoint[3*(Nodes-1)]=std::numeric_limits<double>::quiet_NaN();
    if(fault==3) f.host->slab[1].element[1].history.internal_work_j[0]=std::numeric_limits<double>::max();
    for(unsigned p=0;p<Parents;++p) f.host->assembly.parent[p].status=
        b::ValidResult(f.host->slab[1].element[p],f.host->model.element[p].material,actual.time,actual.epoch)
        ? q::BatchStatus::Success:q::BatchStatus::NonfiniteResult;
    auto& maximum=f.host->assembly.maximum[0];maximum={0,true};
    for(unsigned node=0;node<Nodes;++node) {
      double value=0;if(!b::NodeDisplacement(f.host->model,view,node,value)) maximum.valid=false;
      else if(value>maximum.value) maximum.value=value;
    }
    const bool old_ok=serial::Measure(f.host->model,f.host->slab[0],f.host->slab[1],view,expected);
    EXPECT_EQ(m::Measure(*f.host,f.host->slab[0],f.host->slab[1],view,actual,1),old_ok);
    EXPECT_TRUE(b::SameDiagnostics(actual,expected));
  }
}
} // namespace qbat_gather_test
