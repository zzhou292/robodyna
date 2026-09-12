// SPDX-License-Identifier: MIT
#include "Fixture.h"
#include "../shell_execution/Fixture.h"

namespace self_contact_test {
TEST(SelfContactSurface, ExplicitRigidSkinRetainsGeometryAndZeroMaterialPoints) {
  shell_execution_test::Source source;
  for (auto& q : source.base.geometry.q) q.reference.placement = fe::ShellReferencePlacement::Centered;
  for (auto& t : source.triangles) t.reference.placement = fe::ShellReferencePlacement::Centered;
  fe::ShellBatchBinding shells;
  ASSERT_EQ(shells.InitializeFormulations(source.Geometry()).status, fe::ShellBindingStatus::Success);
  fe::ShellBatchPlasticityBinding catalog;
  ASSERT_EQ(catalog.InitializeExecutionCatalog(shells, source.Catalog()).status, fe::ShellPlasticityBindingStatus::Success);
  std::vector<fe::NodalDomainNode> nodes;
  for (const auto& node : shells.active_nodes()) nodes.push_back({node.source_id, node.position});
  fe::NodalNodeDomain domain;
  ASSERT_TRUE(domain.Initialize({77, nodes.data(), nodes.size()}));
  fe::ShellNodeMap map;
  ASSERT_TRUE(map.Initialize(shells, domain));
  fe::NodalCoefficientLedger ledger;
  ASSERT_TRUE(ledger.Initialize({&map, nullptr, nullptr}));
  const std::uint64_t ids[]{10, 11, 12, 13, 14, 20, 21, 22};
  const fe::rigid::PartTopologyPartInput parts[]{{1000, ids, 5}, {2000, ids + 5, 3}};
  const fe::rigid::PartTopologyMerge merge{1000, 2000};
  fe::rigid::PartTopologyInput input;
  input.source_instance_id = 77;
  input.parts = parts;
  input.part_count = 2;
  input.expected_members = ids;
  input.expected_member_count = 8;
  input.merges = &merge;
  input.merge_count = 1;
  fe::rigid::NodalRigidPartTopology topology;
  ASSERT_TRUE(topology.Initialize(input));
  fe::rigid::NodalRigidPartAssemblyModel model;
  ASSERT_TRUE(model.Initialize(topology, ledger, {1000, .001}));
  fe::NodalRigidAssemblyBinding rigid;
  ASSERT_TRUE(rigid.Initialize(model));
  fe::ShellExecutionBinding execution;
  ASSERT_EQ(execution.Initialize(catalog, ledger, rigid).status, fe::ShellPlasticityBindingStatus::Success);
  fe::ShellBatchFailureBinding failure;
  const auto failures = source.Failures();
  ASSERT_EQ(failure.InitializeExecution(catalog, failures.data(), failures.size()).status,
      fe::ShellPlasticityBindingStatus::Success);
  fe::ShellPhysicalBinding physical;
  ASSERT_TRUE(physical.InitializeExecution({&shells, &catalog, &failure, nullptr}, ledger, execution));
  std::vector<ct::SelfContactParentSelection> selected;
  for (std::size_t row = 0; row < catalog.parent_count(); ++row) {
    const auto& p = *catalog.parent(row);
    selected.push_back({row, p.family, p.family_index, p.source_parent_id, p.source_part_id});
  }
  ct::SelfContactSurfaceBinding surface;
  ASSERT_EQ(surface.Initialize(physical, Input(selected)).status, S::Ok);
  unsigned skin_count = 0;
  for (const auto& parent : surface.parents()) {
    if (parent.law != fe::ShellSectionLaw::RigidSkin) continue;
    ++skin_count;
    EXPECT_EQ(parent.material_points, 0u);
    EXPECT_EQ(parent.reference_half_thickness_m, .001);
  }
  EXPECT_EQ(skin_count, 3u);
  EXPECT_FALSE(surface.OutputDisjoint(execution.parents().data(), sizeof(fe::ShellExecutionParent)));
  EXPECT_FALSE(surface.OutputDisjoint(rigid.members().data(), sizeof(*rigid.members().data())));
}
} // namespace self_contact_test
