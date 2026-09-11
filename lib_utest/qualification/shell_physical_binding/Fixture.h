// SPDX-License-Identifier: MIT
#pragma once
#include <gtest/gtest.h>
#include "../qbat_catalog/Fixture.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include <algorithm>
#include <vector>

namespace shell_physical_test {
namespace fe = tl::fea;
struct Fixture {
  qbat_catalog_test::Fixture source;
  fe::ShellBatchBinding shells;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap map;
  fe::ElementMassContributions point_mass;
  fe::NodalCoefficientLedger ledger;
  explicit Fixture(bool cover_extra = true, double extra_mass = 2) {
    EXPECT_EQ(shells.InitializeFormulations(source.Geometry()).status,fe::ShellBindingStatus::Success);
    EXPECT_EQ(catalog.InitializeFormulations(shells,source.Input()).status,
        fe::ShellPlasticityBindingStatus::Success);
    EXPECT_EQ(failure.Initialize(catalog,source.failures.data(),source.failures.size()).status,
        fe::ShellPlasticityBindingStatus::Success);
    std::vector<fe::NodalDomainNode> nodes{{900001,{3,4,5}}};
    for (std::size_t i = shells.node_count(); i-- > 0;) {
      const auto& node = shells.nodes()[i];
      nodes.push_back({node.source_id,node.position});
    }
    EXPECT_TRUE(domain.Initialize({77,nodes.data(),nodes.size()}));
    EXPECT_TRUE(map.Initialize(shells,domain));
    const fe::ElementMassSource mass{800001,900001,0,extra_mass};
    EXPECT_TRUE(point_mass.Initialize(domain,{77,1,&mass,1}));
    EXPECT_TRUE(ledger.InitializeWithElementMass({{&map,nullptr,nullptr},
        cover_extra ? &point_mass : nullptr}));
  }
  fe::ShellFormulationScope Scope() const { return {&shells,&catalog,&failure,nullptr}; }
};
} // namespace shell_physical_test
