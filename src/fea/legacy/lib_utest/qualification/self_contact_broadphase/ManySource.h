#pragma once
#include "../self_contact_surface/Fixture.h"

namespace surface_broadphase_test {
// Existing qualified typed constructors own all backing. The repeated source
// layers intentionally share nodes: S0 activity/adjacency is not an exclusion.
inline tlfea::contact::SelfContactSurfaceBinding ManySource(std::size_t count) {
  namespace fe = tl::fea;
  namespace ct = tlfea::contact;
  qbat_catalog_test::Fixture seed;
  auto original = seed.geometry.q[0];
  original.reference.placement = fe::ShellReferencePlacement::Centered;
  std::vector<fe::ShellQephBindingInput> quads(count - 1, original);
  std::vector<fe::ShellPlasticityParentInput> parents;
  std::vector<fe::ShellFailureParentInput> failures;
  for (std::size_t i = 0; i + 1 < count; ++i) {
    quads[i].source_parent_id = 10000 + i;
    parents.push_back({fe::ShellBindingFamily::Qeph, i, 10000 + i, 1000, 1000, 1000});
    failures.push_back(qbat_catalog_test::Failure(parents.back()));
  }
  parents.push_back(seed.parents[0]);
  failures.push_back(qbat_catalog_test::Failure(parents.back()));
  fe::ShellBatchBinding shells;
  EXPECT_EQ(shells.InitializeFormulations({{quads.data(), nullptr, count - 1, 0, 4}, &seed.geometry.b, 1}).status,
            fe::ShellBindingStatus::Success);
  std::array<fe::ShellPlasticityMaterialInput, 2> materials{seed.materials[0], seed.materials[2]};
  std::array<fe::ShellPlasticitySectionInput, 2> sections{seed.sections[0], seed.sections[2]};
  auto catalog_input = seed.Input();
  catalog_input.materials = materials.data(); catalog_input.material_count = materials.size();
  catalog_input.sections = sections.data(); catalog_input.section_count = sections.size();
  catalog_input.parents = parents.data(); catalog_input.parent_count = count;
  fe::ShellBatchPlasticityBinding catalog;
  EXPECT_EQ(catalog.InitializeFormulations(shells, catalog_input).status, fe::ShellPlasticityBindingStatus::Success);
  fe::ShellBatchFailureBinding failure;
  EXPECT_EQ(failure.Initialize(catalog, failures.data(), count).status, fe::ShellPlasticityBindingStatus::Success);
  std::vector<fe::NodalDomainNode> nodes;
  for (std::size_t n = 0; n < shells.node_count(); ++n)
    nodes.push_back({shells.nodes()[n].source_id, shells.nodes()[n].position});
  fe::NodalNodeDomain domain;
  EXPECT_TRUE(domain.Initialize({991, nodes.data(), nodes.size()}));
  fe::ShellNodeMap map; EXPECT_TRUE(map.Initialize(shells, domain));
  fe::NodalCoefficientLedger ledger; EXPECT_TRUE(ledger.Initialize({&map, nullptr, nullptr}));
  fe::ShellPhysicalBinding physical; EXPECT_TRUE(physical.Initialize({&shells, &catalog, &failure, nullptr}, ledger));
  std::vector<ct::SelfContactParentSelection> selected;
  for (std::size_t i = 0; i < count; ++i)
    selected.push_back({i, parents[i].family, parents[i].family_index, parents[i].source_parent_id, parents[i].source_part_id});
  ct::SelfContactSurfaceBinding source;
  EXPECT_EQ(source.Initialize(physical, self_contact_test::Input(selected)).status, ct::SelfContactSurfaceStatus::Ok);
  return source;
}
} // namespace surface_broadphase_test
