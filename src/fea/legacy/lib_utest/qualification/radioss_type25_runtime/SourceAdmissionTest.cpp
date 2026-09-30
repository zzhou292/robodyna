// SPDX-License-Identifier: AGPL-3.0-or-later
#include "SourceAdmissionFixture.h"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <type_traits>
namespace type25_source_test {
namespace {
void Unchanged(const rd::SourceStaging& output) {
  EXPECT_EQ(output.bytes,991u);EXPECT_EQ(output.ids,std::vector<std::uint64_t>{88});
  EXPECT_FALSE(output.moving.enabled);EXPECT_EQ(output.moving.free_main_ids,std::vector<std::uint32_t>{71});
}
rd::SourceStaging Sentinel() {
  rd::SourceStaging output;output.bytes=991;output.ids={88};output.moving.free_main_ids={71};return output;
}
}
TEST(NativeType25Source, MovingTopologyRetainsPhysicalIdentityAndCompleteNativeFreeRoster) {
  static_assert(!std::is_convertible_v<n::MovingMainSource,n::FixedMainSource>);
  Fixture f;const auto source=f.Source();rd::SourceStaging result;
  const auto report=rd::PrepareSource(f.Config(),source,f.physical.physical,{},result);
  ASSERT_EQ(report.status,n::TransactionStatus::Ok)<<report.message;
  ASSERT_TRUE(result.moving.enabled);EXPECT_EQ(result.moving.free_main_ids,(std::vector<std::uint32_t>{1,2,3,4}));
  EXPECT_EQ(result.moving.main_coefficients,(std::vector<double>{1e6,1e6,1e6,1e6}));
  EXPECT_EQ(result.moving.starter.profile,s::Profile::OrdinaryExteriorMovingMain);
  EXPECT_EQ(result.moving.starter.topology,s::TopologyPolicy::ManifoldTwoSided);
  EXPECT_EQ(result.moving.starter.source_generation,7u);EXPECT_EQ(result.moving.topology.mains,f.starter.mains);
  EXPECT_EQ(result.moving.topology.main_count,4u);EXPECT_EQ(result.moving.topology.normal_to_main.entry_count,14u);
  ASSERT_EQ(result.primary.size(),2u);EXPECT_EQ(result.primary[0].nodes[0],0u);EXPECT_EQ(result.primary[1].nodes[0],4u);
  // Source arrays are immutable borrowed startup operands. No owned values are
  // synthesized from a captured trajectory or precomputed reference force.
  EXPECT_EQ(result.inventory.node_ids,result.ids.data());EXPECT_EQ(result.maintenance.main_nodes,result.main_nodes.data());
}
TEST(NativeType25Source, FixedAdmissionStillRejectsMovingNodesAndAllowsFullyFixedSource) {
  Fixture f;n::FixedMainSource fixed;static_cast<n::ContactSourceInput&>(fixed)=f.Source();
  auto output=Sentinel();EXPECT_EQ(rd::PrepareSource(f.Config(),fixed,f.physical.physical,{},output).status,n::TransactionStatus::SourceMismatch);
  Unchanged(output);for(auto& node:f.nodes)node.constraint=7;
  ASSERT_EQ(rd::PrepareSource(f.Config(),fixed,f.physical.physical,{},output).status,n::TransactionStatus::Ok);
  EXPECT_FALSE(output.moving.enabled);EXPECT_TRUE(output.moving.free_main_ids.empty());
}
TEST(NativeType25Source, FreshRosterOmitsInactiveMainsWithoutChangingTopologyOrDomain) {
  Fixture f;f.mains[0].coefficient=0;f.mains[2].coefficient=0;
  auto source=f.Source();rd::SourceStaging output;
  ASSERT_EQ(rd::PrepareSource(f.Config(),source,f.physical.physical,{},output).status,n::TransactionStatus::Ok);
  EXPECT_EQ(output.moving.free_main_ids,(std::vector<std::uint32_t>{2,4}));
  EXPECT_EQ(output.moving.main_coefficients,(std::vector<double>{0,1e6,0,1e6}));
  EXPECT_EQ(output.moving.topology.main_count,4u);EXPECT_EQ(output.inventory.mains,2u);
}
TEST(NativeType25Source, SnapshotCorruptionRejectsBeforePublishingAnyStartupState) {
  Fixture f;
  for(unsigned field=0;field<13;++field) {
    SCOPED_TRACE(field);auto source=f.Source();auto output=Sentinel();
    std::vector<s::Main> mains(f.starter.mains,f.starter.mains+f.starter.main_count);
    std::vector<std::uint32_t> expanded(f.starter.expanded_to_primary,f.starter.expanded_to_primary+f.starter.main_count);
    std::vector<std::uint32_t> partners(f.starter.primary_to_partner,f.starter.primary_to_partner+f.starter.primary_count);
    std::vector<n::StoredNormal> normals(f.starter.starter.face_normals,f.starter.starter.face_normals+4*f.starter.main_count);
    std::vector<s::NormalReference> references(f.starter.starter.references,f.starter.starter.references+f.starter.starter.reference_count);
    source.starter.mains=mains.data();source.starter.expanded_to_primary=expanded.data();
    source.starter.primary_to_partner=partners.data();source.starter.starter.face_normals=normals.data();
    source.starter.starter.references=references.data();
    if(field==0)++source.starter.source_generation;
    if(field==1)++mains[0].source_id;
    if(field==2)expanded[3]=0;
    if(field==3)partners[0]=4;
    if(field==4)mains[0].normal_reference[1]=mains[0].normal_reference[0];
    if(field==5)mains[0].neighbor_edges[0]=1; // No neighboring edge exists.
    if(field==6)normals[0].z=-normals[0].z;
    if(field==7)references[0].boundary=2; // Ready-stage cache is not a Starter boolean.
    if(field==8)references[0].bisector[0].x=std::numeric_limits<float>::quiet_NaN();
    if(field==9)source.starter.normal_incidence_count=SIZE_MAX;
    if(field==10)source.starter.mains=reinterpret_cast<const s::Main*>(reinterpret_cast<const unsigned char*>(mains.data())+1);
    if(field==11)source.starter.profile=s::Profile::OrdinaryExteriorFixedMain;
    if(field==12)source.starter.topology=static_cast<s::TopologyPolicy>(99);
    EXPECT_NE(rd::PrepareSource(f.Config(),source,f.physical.physical,{},output).status,n::TransactionStatus::Ok);
    Unchanged(output);
  }
}
TEST(NativeType25Source, LiteralUnusedTriangleZeroBitsAndBoundaryBisectorsRemainAuthenticated) {
  Fixture f;auto source=f.Source();auto output=Sentinel();
  const auto original=f.mains[1].normal_slot[2].x;
  ASSERT_EQ(original,0.f);f.mains[1].normal_slot[2].x=std::copysign(0.f,-std::copysign(1.f,original));
  EXPECT_EQ(rd::PrepareSource(f.Config(),source,f.physical.physical,{},output).status,n::TransactionStatus::SourceMismatch);
  Unchanged(output);f.mains[1].normal_slot[2].x=original;
  const auto reference=f.references[0];f.references[0].bisector[0].x+=1;
  EXPECT_EQ(rd::PrepareSource(f.Config(),source,f.physical.physical,{},output).status,n::TransactionStatus::SourceMismatch);
  Unchanged(output);f.references[0]=reference;
  EXPECT_EQ(rd::PrepareSource(f.Config(),source,f.physical.physical,{},output).status,n::TransactionStatus::Ok);
}
TEST(NativeType25Source, ExactHostCapsAndUnsupportedActivationPreserveOutputOnRetry) {
  Fixture f;auto source=f.Source();rd::SourceStaging valid;
  ASSERT_EQ(rd::PrepareSource(f.Config(),source,f.physical.physical,{},valid).status,n::TransactionStatus::Ok);
  n::TransactionLimits limits;limits.max_host_bytes=valid.bytes;rd::SourceStaging exact;
  ASSERT_EQ(rd::PrepareSource(f.Config(),source,f.physical.physical,limits,exact).status,n::TransactionStatus::Ok);
  --limits.max_host_bytes;
  for(unsigned retry=0;retry<2;++retry) {
    auto output=Sentinel();EXPECT_EQ(rd::PrepareSource(f.Config(),source,f.physical.physical,limits,output).status,n::TransactionStatus::ResourceLimit);Unchanged(output);
  }
  for(unsigned field=0;field<6;++field) {
    SCOPED_TRACE(field);auto invalid=source;auto output=Sentinel();
    if(field==0)invalid.activation.edge_mode=1;
    if(field==1)invalid.activation.foreign_rows=1;
    if(field==2)invalid.activation.partitions=2;
    if(field==3)invalid.activation.neighbor_removal=1;
    if(field==4)invalid.activation.local_processor=2;
    if(field==5)invalid.activation.free_roster=n::normal_activation::FreeRosterPolicy::Unspecified;
    EXPECT_EQ(rd::PrepareSource(f.Config(),invalid,f.physical.physical,{},output).status,n::TransactionStatus::UnsupportedProfile);Unchanged(output);
  }
}
TEST(NativeType25Source, NonzeroUnimplementedOptimizationControlsRejectBothProfiles) {
  Fixture f;for(auto& node:f.nodes)node.constraint=7;
  for(unsigned field=0;field<2;++field)for(double value:{1e-100,1.}) {
    auto moving=f.Source();if(field==0)moving.drad=value;else moving.gap_load=value;
    n::FixedMainSource fixed;static_cast<n::ContactSourceInput&>(fixed)=moving;
    auto output=Sentinel();
    EXPECT_EQ(rd::PrepareSource(f.Config(),fixed,f.physical.physical,{},output).status,n::TransactionStatus::UnsupportedProfile);Unchanged(output);
    EXPECT_EQ(rd::PrepareSource(f.Config(),moving,f.physical.physical,{},output).status,n::TransactionStatus::UnsupportedProfile);Unchanged(output);
  }
  auto moving=f.Source();moving.drad=-0.;moving.gap_load=-0.;
  n::FixedMainSource fixed;static_cast<n::ContactSourceInput&>(fixed)=moving;
  rd::SourceStaging output;
  EXPECT_EQ(rd::PrepareSource(f.Config(),fixed,f.physical.physical,{},output).status,n::TransactionStatus::Ok);
  EXPECT_EQ(rd::PrepareSource(f.Config(),moving,f.physical.physical,{},output).status,n::TransactionStatus::Ok);
}
} // namespace type25_source_test
