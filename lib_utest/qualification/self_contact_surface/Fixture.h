// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_catalog/Fixture.h"
#include "lib_src/collision/SelfContactSurfaceBinding.h"
#include <vector>

namespace self_contact_test {
namespace fe = tl::fea;
namespace ct = tlfea::contact;
using S = ct::SelfContactSurfaceStatus;
struct Fixture {
  qbat_catalog_test::Fixture source;
  fe::ShellBatchBinding shells;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap map;
  fe::NodalCoefficientLedger ledger;
  fe::ShellPhysicalBinding physical;
  explicit Fixture(bool centered = true, std::uint64_t instance = 77, bool distinct_coincident_layer = false) {
    if (centered) {
      for (auto& parent : source.geometry.q) parent.reference.placement = fe::ShellReferencePlacement::Centered;
      for (auto& parent : source.triangles) parent.reference.placement = fe::ShellReferencePlacement::Centered;
    }
    auto geometry = source.Geometry();
    if (distinct_coincident_layer) {
      for (unsigned i = 0; i < 4; ++i) {
        source.geometry.q[1].nodes[i] = 5 + i;
        source.geometry.q[1].reference.node_ids[i] = 20 + i;
      }
      geometry.shells.node_count = 9;
    }
    EXPECT_EQ(shells.InitializeFormulations(geometry).status, fe::ShellBindingStatus::Success);
    EXPECT_EQ(catalog.InitializeFormulations(shells, source.Input()).status, fe::ShellPlasticityBindingStatus::Success);
    EXPECT_EQ(failure.Initialize(catalog, source.failures.data(), source.failures.size()).status,
        fe::ShellPlasticityBindingStatus::Success);
    std::vector<fe::NodalDomainNode> nodes;
    for (std::size_t n = shells.node_count(); n-- > 0;)
      nodes.push_back({shells.nodes()[n].source_id, shells.nodes()[n].position});
    EXPECT_TRUE(domain.Initialize({instance, nodes.data(), nodes.size()}));
    EXPECT_TRUE(map.Initialize(shells, domain));
    EXPECT_TRUE(ledger.Initialize({&map, nullptr, nullptr}));
    EXPECT_TRUE(physical.Initialize({&shells, &catalog, &failure, nullptr}, ledger));
  }
  std::vector<ct::SelfContactParentSelection> Selection() const {
    std::vector<ct::SelfContactParentSelection> out;
    for (std::size_t row = 0; row < catalog.parent_count(); ++row) {
      const auto& p = *catalog.parent(row);
      out.push_back({row, p.family, p.family_index, p.source_parent_id, p.source_part_id});
    }
    return out;
  }
};
inline ct::SelfContactSurfaceInput Input(const std::vector<ct::SelfContactParentSelection>& rows) {
  return {rows.data(), rows.size()};
}
} // namespace self_contact_test
