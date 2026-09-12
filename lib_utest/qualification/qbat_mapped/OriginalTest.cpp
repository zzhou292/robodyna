// SPDX-License-Identifier: MIT
#include "../qbat_catalog/OriginalCatalog.h"
#include "lib_src/elements/qbat/mapped/Startup.h"
#include "lib_src/elements/qbat/QbatBatchResultChecks.h"

namespace qbat_mapped_original_test {
namespace fe=tl::fea;
namespace qb=fe::qbat;
namespace mapped=qb::mapped;
namespace batch=qb::batch_detail;
using qbat_binding_test::Bits;

TEST(QbatMappedOriginal,All4250QuadsKeepSourceOrderAcrossScrambledLargerPhysicalDomain) {
  qbat_catalog_test::SourceCatalog source;
  fe::ShellBatchBinding shells;
  ASSERT_EQ(shells.InitializeFormulations(source.geometry.Input(),fe::ShellHostBindingLimits::Vehicle()).status,
      fe::ShellBindingStatus::Success);
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeFormulationCatalog(shells,source.Input(),fe::ShellPlasticityCatalogLimits::Vehicle()).status,
      fe::ShellPlasticityBindingStatus::Success);
  fe::ShellBatchFailureBinding failure;
  ASSERT_EQ(failure.Initialize(catalog,source.failures.data(),source.failures.size(),
      fe::ShellBatchFailureLimits::Vehicle()).status,fe::ShellPlasticityBindingStatus::Success);
  // The additional point-mass row is a declared synthetic mapping control,
  // not an original Yaris source card or a full-vehicle coverage claim.
  std::vector<fe::NodalDomainNode> nodes{{9000000000ull,{0,0,0}}};
  for (std::size_t i=shells.node_count();i>0;--i) {
    const auto& node=shells.nodes()[i-1];
    nodes.push_back({node.source_id,node.position});
  }
  fe::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({91,nodes.data(),nodes.size()},fe::NodalDomainLimits::Vehicle()));
  fe::ShellNodeMap mapping;
  ASSERT_TRUE(mapping.Initialize(shells,domain,fe::ShellNodeMapLimits::Vehicle()));
  ASSERT_FALSE(mapping.identity_map());
  const fe::ElementMassSource extra{9000000001ull,nodes[0].source_id,0,.002};
  fe::ElementMassContributions point;
  EXPECT_EQ(point.Initialize(domain,{92,1000,&extra,1}).status,fe::NodalDomainStatus::InvalidInput);
  ASSERT_TRUE(point.Initialize(domain,{domain.source_instance_id(),1000,&extra,1}));
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.InitializeWithElementMass({{&mapping},&point},fe::CoefficientLimits::Vehicle()));
  fe::ShellPhysicalBinding physical;
  ASSERT_TRUE(physical.Initialize({&shells,&catalog,&failure,nullptr},ledger,
      fe::ShellPhysicalBindingLimits::Vehicle()));
  qb::BatchConfig config;
  config.owner.owner_id=71;
  config.owner.node_count=nodes.size();
  config.owner.fixed_dt=0x1p-20;
  config.owner.has_rotations=true;
  config.owner.temporal_scheme=fe::NodalTemporalScheme::StaggeredHalfKickStart;
  config.configuration_id=101;
  config.qualification_id=102;
  config.element_count=shells.qbat_count();
  config.usage=qb::BatchUsage::CoupledForces;
  config.storage_limits=fe::ShellResidentLimits::Vehicle();
  config.max_device_bytes=fe::MaxVehicleShellResidentDeviceBytes;
  batch::Layout layout;
  // BuildModel owns the mapped incidence too, matching the production forecast.
  ASSERT_TRUE(layout.InitializeMapped(config.element_count,nodes.size(),catalog.curve_point_count(),config.max_device_bytes));
  tl::util::HostArena arena;
  ASSERT_TRUE(arena.Initialize(layout.bytes));
  auto* storage=layout.Construct(arena);
  ASSERT_NE(storage,nullptr);
  ASSERT_NE(storage->assembly.offsets,nullptr);
  ASSERT_NE(storage->assembly.incidence,nullptr);
  qb::BatchDiagnostics diagnostics;
  ASSERT_EQ(mapped::BuildModel(config,physical,*storage,diagnostics).status,qb::BatchStatus::Success);
  ASSERT_EQ(shells.qeph_count(),0u);
  ASSERT_EQ(shells.t3_count(),1u);
  ASSERT_EQ(config.element_count,4250u);
  EXPECT_EQ(storage->model.mass[0],2);
  EXPECT_EQ(storage->model.inertia[0],0);
  EXPECT_EQ(storage->assembly.offsets[0],0u);
  EXPECT_EQ(storage->assembly.offsets[1],0u); // Extra point mass has no shell incidence.
  EXPECT_EQ(storage->assembly.offsets[nodes.size()],4*config.element_count);
  for (std::size_t node=0;node<nodes.size();++node) {
    SCOPED_TRACE(node);
    EXPECT_EQ(Bits(storage->model.mass[node]),Bits(ledger.nodes()[node].coefficients.mass));
    EXPECT_EQ(Bits(storage->model.inertia[node]),Bits(ledger.nodes()[node].coefficients.isotropic_inertia));
    const auto begin=storage->assembly.offsets[node],end=storage->assembly.offsets[node+1];
    ASSERT_LE(begin,end);
    ASSERT_LE(end,4*config.element_count);
    for (auto entry=begin;entry<end;++entry) {
      const auto key=storage->assembly.incidence[entry];
      ASSERT_LT(key,4*config.element_count);
      EXPECT_EQ(storage->model.element[key/4].nodes[key%4],node);
      if (entry>begin) EXPECT_LT(storage->assembly.incidence[entry-1],key);
    }
  }
  for (std::size_t parent=0;parent<config.element_count;++parent) {
    SCOPED_TRACE(parent);
    const auto& element=storage->model.element[parent];
    const auto& original=source.geometry.quads[parent];
    EXPECT_EQ(element.source_parent_id,original.source_parent_id);
    for (unsigned slot=0;slot<4;++slot) {
      EXPECT_EQ(element.nodes[slot],mapping.owner_index(original.nodes[slot]));
    }
    const auto& row=storage->slab[0].element[parent];
    ASSERT_TRUE(batch::ValidResult(row,element.material,0,0));
    EXPECT_EQ(row.stamp.sample_index,0u);
    EXPECT_EQ(row.diagnostics.translation_stiffness_n_m,0);
  }
  std::printf("Original mapped QBAT: parents=%zu shell_nodes=%zu physical_nodes=%zu arena_bytes=%zu retained_scope=%zu\n",
      config.element_count,shells.node_count(),nodes.size(),layout.bytes,physical.owned_payload_bytes());
}
} // namespace qbat_mapped_original_test
