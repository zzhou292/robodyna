// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_catalog/Fixture.h"
#include "lib_src/assembly/ShellPhysicalBinding.h"
#include <algorithm>

namespace shell_execution_test {
namespace fe = tl::fea;
using Status = fe::ShellPlasticityBindingStatus;
using Family = fe::ShellBindingFamily;
using Law = fe::ShellSectionLaw;
using qbat_binding_test::Bits;

// One explicit rigid skin shares nodes with ordinary constitutive glass and
// QBAT. A disjoint original child PART is merged into its primary. Thus root
// membership alone cannot authorize swapping their original PIDs.
struct Source {
  qbat_catalog_test::Fixture base;
  std::array<fe::ShellT3BindingInput,4> triangles;
  std::array<fe::ShellPlasticityParentInput,7> parents;
  Source() {
    std::copy(base.triangles.begin(),base.triangles.end(),triangles.begin());
    std::copy(base.parents.begin(),base.parents.end(),parents.begin());
    auto& m = base.materials[0];
    m.curve_id = 0;
    m.law = Law::RigidSkin;
    base.sections[0].through_thickness_points = 0;
    base.sections[0].formulation = fe::ShellSectionFormulation::Nonconstitutive;
    triangles[3] = triangles[0];
    auto& child = triangles[3];
    child.source_parent_id = 300;
    child.nodes = {5,6,7};
    const tl::math::Vec3 x[]{{.2,0,0},{.24,0,0},{.2,.02,0}};
    for (unsigned n = 0; n < 3; ++n) {
      child.reference.node_ids[n] = 20+n;
      child.reference.position[n] = x[n];
    }
    parents[6] = {Family::T3,3,300,2000,1000,1000};
  }
  fe::ShellFormulationCollectionInput Geometry() const {
    return {{base.geometry.q.data(),triangles.data(),2,4,8},&base.geometry.b,1};
  }
  fe::ShellBatchPlasticityBindingInput Catalog() const {
    return {&base.curve,base.materials.data(),base.sections.data(),parents.data(),1,3,3,7};
  }
  std::array<fe::ShellFailureParentInput,7> Failures() const {
    std::array<fe::ShellFailureParentInput,7> rows;
    for (std::size_t i = 0; i < rows.size(); ++i) {
      rows[i].source = parents[i];
      if (parents[i].material_id != 1000) {
        rows[i].policy = fe::ShellFailurePolicy::ConstantAllPoints;
        rows[i].constant.failure_strain = 2.5;
      }
    }
    return rows;
  }
};
struct Fixture {
  Source source;
  fe::ShellBatchBinding shells;
  fe::ShellBatchPlasticityBinding catalog;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap mapping;
  fe::NodalCoefficientLedger ledger;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidAssemblyBinding rigid;
  explicit Fixture(bool merge = true) {
    EXPECT_EQ(shells.InitializeFormulations(source.Geometry()).status,fe::ShellBindingStatus::Success);
    EXPECT_EQ(catalog.InitializeExecutionCatalog(shells,source.Catalog()).status,Status::Success);
    std::vector<fe::NodalDomainNode> nodes;
    for (std::size_t i = shells.node_count(); i-- > 0;) {
      const auto& node = shells.nodes()[i];
      nodes.push_back({node.source_id,node.position});
    }
    EXPECT_TRUE(domain.Initialize({77,nodes.data(),nodes.size()}));
    EXPECT_TRUE(mapping.Initialize(shells,domain));
    EXPECT_TRUE(ledger.Initialize({&mapping,nullptr,nullptr}));
    const std::array<std::uint64_t,8> ids{{10,11,12,13,14,20,21,22}};
    const fe::rigid::PartTopologyPartInput p[]{{1000,ids.data(),5},{2000,ids.data()+5,3}};
    const fe::rigid::PartTopologyMerge joined{1000,2000};
    fe::rigid::PartTopologyInput input;
    input.source_instance_id = 77;
    input.parts = p;
    input.part_count = 2;
    input.expected_members = ids.data();
    input.expected_member_count = ids.size();
    input.merges = merge ? &joined : nullptr;
    input.merge_count = merge ? 1 : 0;
    fe::rigid::NodalRigidPartTopology topology;
    EXPECT_TRUE(topology.Initialize(input));
    EXPECT_TRUE(parts.Initialize(topology,ledger,{1000,.001}));
    EXPECT_TRUE(rigid.Initialize(parts));
  }
};
} // namespace shell_execution_test
