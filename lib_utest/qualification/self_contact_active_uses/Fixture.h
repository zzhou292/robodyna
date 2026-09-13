// SPDX-License-Identifier: MIT
#pragma once
#include "../qbat_catalog/Fixture.h"
#include "lib_src/collision/SelfContactActiveUseBinding.h"
#include "lib_src/collision/FixedContactFacetValues.h"
#include "lib_src/constraints/NodalRigidPartTopology.h"
#include "lib_src/constraints/tied_shell/TiedPostKinChk.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <array>
#include <memory>
#include <numeric>
#include <vector>

namespace active_use_test {
namespace c = tlfea::contact;
namespace fe = tl::fea;
namespace tied = tl::constraints::tied_shell;
namespace cin = tied::cin;
using Code = c::SelfContactActiveUseStatus;

struct Fixture {
  qbat_catalog_test::Fixture source;
  fe::ShellBatchBinding shells;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap map;
  fe::NodalCoefficientLedger ledger;
  fe::ShellPhysicalBinding physical;
  c::SelfContactSurfaceBinding surface;
  c::FixedContactFacetBinding facets;
  std::vector<c::SelfContactParentSelection> selection;

  explicit Fixture(unsigned level = 0, bool warped = false,
      bool distinct = false, bool reverse_selection = false) {
    for (auto& q : source.geometry.q)
      q.reference.placement = fe::ShellReferencePlacement::Centered;
    for (auto& t : source.triangles)
      t.reference.placement = fe::ShellReferencePlacement::Centered;
    if (warped) {
      auto warp = [](auto& reference) {
        for (unsigned i = 0;
             i < sizeof(reference.node_ids)/sizeof(reference.node_ids[0]); ++i)
          if (reference.node_ids[i] == 12) reference.position[i].z = .007;
      };
      for (auto& q : source.geometry.q) warp(q.reference);
      warp(source.geometry.b.reference.quadrilateral);
      for (auto& t : source.triangles) warp(t.reference);
    }
    auto geometry = source.Geometry();
    if (distinct) {
      for (unsigned i = 0; i < 4; ++i) {
        source.geometry.q[1].nodes[i] = 5+i;
        source.geometry.q[1].reference.node_ids[i] = 20+i;
      }
      for (unsigned i = 0; i < 3; ++i) {
        source.triangles[1].nodes[i] = 5+i;
        source.triangles[1].reference.node_ids[i] = 20+i;
        source.triangles[1].reference.position[i] =
            source.geometry.q[1].reference.position[i];
      }
      geometry.shells.node_count = 9;
    }
    EXPECT_EQ(shells.InitializeFormulations(geometry).status,
        fe::ShellBindingStatus::Success);
    EXPECT_EQ(catalog.InitializeFormulations(shells, source.Input()).status,
        fe::ShellPlasticityBindingStatus::Success);
    EXPECT_EQ(failure.Initialize(catalog, source.failures.data(),
        source.failures.size()).status, fe::ShellPlasticityBindingStatus::Success);
    std::vector<fe::NodalDomainNode> nodes;
    for (std::size_t n = shells.node_count(); n-- > 0;) {
      const auto& node = shells.nodes()[n];
      nodes.push_back({node.source_id, node.position});
    }
    EXPECT_TRUE(domain.Initialize({77, nodes.data(), nodes.size()}));
    EXPECT_TRUE(map.Initialize(shells, domain));
    EXPECT_TRUE(ledger.Initialize({&map, nullptr, nullptr}));
    EXPECT_TRUE(physical.Initialize({&shells, &catalog, &failure, nullptr}, ledger));
    for (std::size_t row = 0; row < catalog.parent_count(); ++row) {
      const auto& p = *catalog.parent(row);
      selection.push_back({row, p.family, p.family_index,
          p.source_parent_id, p.source_part_id});
    }
    if (reverse_selection) std::reverse(selection.begin(), selection.end());
    const c::SelfContactSurfaceInput input{selection.data(), selection.size()};
    EXPECT_EQ(surface.Initialize(physical, input).status,
        c::SelfContactSurfaceStatus::Ok);
    EXPECT_EQ(facets.Initialize(surface, {{}, level}).status,
        c::FixedContactFacetStatus::Ok);
  }
  std::size_t Parent(std::uint64_t eid, const c::SelfContactActiveUseBinding& uses) const {
    for (std::size_t p = 0; p < uses.parents().size(); ++p)
      if (uses.parents()[p].source.source_parent_id == eid) return p;
    return SIZE_MAX;
  }
  std::size_t VertexUse(std::uint64_t eid, const c::SelfContactActiveUseBinding& uses,
      std::uint64_t source_nid = 0) const {
    const auto p = Parent(eid, uses);
    for (std::size_t i = 0; i < uses.vertex_uses().size(); ++i) {
      const auto& use = uses.vertex_uses()[i];
      if (use.parent != p) continue;
      if (!source_nid || (use.key.kind == c::FacetVertexKind::SourceVertex &&
          use.key.first == source_nid)) return i;
    }
    return SIZE_MAX;
  }
  std::size_t RemoteFacet(std::uint64_t eid, std::uint32_t vertex_feature,
      const c::SelfContactActiveUseBinding& uses) const {
    const auto p = Parent(eid, uses);
    const auto& parent = uses.parents()[p];
    for (std::size_t i = parent.facet_offset;
         i < std::size_t(parent.facet_offset)+parent.facet_count; ++i) {
      bool incident = false;
      for (const auto v : uses.facet_uses()[i].vertex_features)
        incident = incident || v == vertex_feature;
      if (!incident) return i;
    }
    return SIZE_MAX;
  }
  c::WeightedSurfacePoint FacePoint(std::size_t facet_index,
      const c::SelfContactActiveUseBinding& uses,
      std::array<double,3> barycentric = {{1./3,1./3,1./3}}) const {
    const auto& use = uses.facet_uses()[facet_index];
    const auto& parent = uses.parents()[use.parent];
    c::FixedContactFacet facet;
    EXPECT_EQ(facets.Describe(parent.surface_parent, use.local_facet, &facet).status,
        c::FixedContactFacetStatus::Ok);
    c::WeightedSurfacePoint point;
    EXPECT_EQ(c::ComposeFacetPoint(facet, barycentric.data(), domain.node_count(), &point),
        c::Status::kOk);
    return point;
  }
  std::vector<std::uint8_t> Active(const c::SelfContactActiveUseBinding& uses,
      std::uint8_t value = 1) const {
    return std::vector<std::uint8_t>(uses.parents().size(), value);
  }
};

enum class ExecutionRigidMode { MergedParts, SeparateParts, PartAndPlain };

struct ExecutionSource {
  qbat_catalog_test::Fixture base;
  std::array<fe::ShellT3BindingInput,4> triangles;
  std::array<fe::ShellPlasticityParentInput,7> parents;
  std::size_t node_count=9;
  ExecutionSource() {
    std::copy(base.triangles.begin(),base.triangles.end(),triangles.begin());
    std::copy(base.parents.begin(),base.parents.end(),parents.begin());
    auto& material=base.materials[0];
    material.curve_id=0;
    material.law=fe::ShellSectionLaw::RigidSkin;
    base.sections[0].through_thickness_points=0;
    base.sections[0].formulation=fe::ShellSectionFormulation::Nonconstitutive;
    triangles[3]=triangles[0];
    triangles[3].source_parent_id=300;
    triangles[3].nodes={5,6,7};
    const tl::math::Vec3 x[]{{.2,0,0},{.24,0,0},{.2,.02,0}};
    for (unsigned n=0;n<3;++n) {
      triangles[3].reference.node_ids[n]=20+n;
      triangles[3].reference.position[n]=x[n];
    }
    triangles[2].nodes={0,5,8};
    triangles[2].reference.node_ids[0]=10;
    triangles[2].reference.node_ids[1]=20;
    triangles[2].reference.node_ids[2]=32;
    triangles[2].reference.position[0]=base.geometry.q[0].reference.position[0];
    triangles[2].reference.position[1]=x[0];
    triangles[2].reference.position[2]={.4,.02,0};
    parents[6]={fe::ShellBindingFamily::T3,3,300,2000,1000,1000};
  }
  fe::ShellFormulationCollectionInput Geometry() const {
    return {{base.geometry.q.data(),triangles.data(),2,4,node_count},
        &base.geometry.b,1};
  }
  fe::ShellBatchPlasticityBindingInput Catalog() const {
    return {&base.curve,base.materials.data(),base.sections.data(),
        parents.data(),1,3,3,7};
  }
  std::array<fe::ShellFailureParentInput,7> Failures() const {
    std::array<fe::ShellFailureParentInput,7> rows;
    for (std::size_t i=0;i<rows.size();++i) {
      rows[i].source=parents[i];
      if (parents[i].material_id != 1000) {
        rows[i].policy=fe::ShellFailurePolicy::ConstantAllPoints;
        rows[i].constant.failure_strain=2.5;
      }
    }
    return rows;
  }
};

// Qualification fixture whose contact surface retains the exact rigid
// authority used by ShellPhysicalBinding::InitializeExecution.
struct ExecutionFixture {
  ExecutionSource source;
  fe::ShellBatchBinding shells;
  fe::ShellBatchPlasticityBinding catalog;
  fe::ShellBatchFailureBinding failure;
  fe::NodalNodeDomain domain;
  fe::ShellNodeMap map;
  fe::NodalCoefficientLedger ledger;
  fe::rigid::NodalRigidPartTopology topology;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidGroupModel plain;
  fe::NodalRigidAssemblyBinding rigid;
  fe::ShellExecutionBinding execution;
  fe::ShellPhysicalBinding physical;
  c::SelfContactSurfaceBinding surface;
  c::FixedContactFacetBinding facets;
  std::vector<c::SelfContactParentSelection> selection;
  std::uint64_t second_plain_parent = 0;
  std::uint64_t mixed_parent = 0;

  explicit ExecutionFixture(unsigned level, ExecutionRigidMode mode) {
    for (auto& q : source.base.geometry.q)
      q.reference.placement = fe::ShellReferencePlacement::Centered;
    for (auto& t : source.triangles)
      t.reference.placement = fe::ShellReferencePlacement::Centered;
    if (mode == ExecutionRigidMode::PartAndPlain) {
      // The added child and one native T3 are ordinary execution parents on
      // the same plain rigid group. Numeric source ID 1000 deliberately
      // collides with the independent PART source ID 1000.
      source.parents[6].material_id = 1001;
      source.parents[6].section_id = 1001;
      const auto child = source.triangles[3];
      source.triangles[2].nodes = child.nodes;
      source.triangles[2].reference = child.reference;
      source.node_count=8;
      for (auto& parent : source.parents) {
        if (parent.family == fe::ShellBindingFamily::T3 &&
            parent.family_index == 2) {
          parent.source_part_id = 3000;
          parent.material_id = 1001;
          parent.section_id = 1001;
          second_plain_parent = parent.source_parent_id;
        }
      }
    }
    for (const auto& parent : source.parents)
      if (parent.family == fe::ShellBindingFamily::T3 &&
          parent.family_index == 2)
        mixed_parent=parent.source_parent_id;
    EXPECT_EQ(shells.InitializeFormulations(source.Geometry()).status,
        fe::ShellBindingStatus::Success);
    EXPECT_EQ(catalog.InitializeExecutionCatalog(shells, source.Catalog()).status,
        fe::ShellPlasticityBindingStatus::Success);
    std::vector<fe::NodalDomainNode> nodes;
    for (std::size_t n = shells.node_count(); n-- > 0;) {
      const auto& node = shells.nodes()[n];
      nodes.push_back({node.source_id, node.position});
    }
    EXPECT_TRUE(domain.Initialize({77, nodes.data(), nodes.size()}));
    EXPECT_TRUE(map.Initialize(shells, domain));
    EXPECT_TRUE(ledger.Initialize({&map, nullptr, nullptr}));
    const std::array<std::uint64_t,8> ids{{10,11,12,13,14,20,21,22}};
    const std::array<std::uint64_t,8> expected{{10,11,12,13,14,20,21,22}};
    const fe::rigid::PartTopologyPartInput declarations[]{
        {1000, ids.data(), 5}, {2000, ids.data()+5, 3}};
    const fe::rigid::PartTopologyMerge merge{1000,2000};
    fe::rigid::PartTopologyInput input;
    input.source_instance_id = 77;
    input.parts = declarations;
    input.part_count = mode == ExecutionRigidMode::PartAndPlain ? 1 : 2;
    input.expected_members = expected.data();
    input.expected_member_count =
        mode == ExecutionRigidMode::PartAndPlain ? 5 : expected.size();
    input.other_rigid_members =
        mode == ExecutionRigidMode::PartAndPlain ? ids.data()+5 : nullptr;
    input.other_rigid_member_count =
        mode == ExecutionRigidMode::PartAndPlain ? 3 : 0;
    input.merges = mode == ExecutionRigidMode::MergedParts ? &merge : nullptr;
    input.merge_count = mode == ExecutionRigidMode::MergedParts ? 1 : 0;
    EXPECT_TRUE(topology.Initialize(input));
    EXPECT_TRUE(parts.Initialize(topology, ledger, {1000,.001}));
    if (mode == ExecutionRigidMode::PartAndPlain) {
      std::array<fe::NodalRigidGroupMember,3> members;
      for (unsigned i = 0; i < members.size(); ++i) {
        const auto node = domain.Find(ids[5+i]);
        const auto& coefficient = ledger.nodes()[node].coefficients;
        members[i] = {ids[5+i], node, domain.nodes()[node].position,
            coefficient.mass, coefficient.isotropic_inertia,
            coefficient.shell.physical_inertia,
            coefficient.shell.added_inertia, 0};
      }
      const fe::NodalRigidGroupInput group{
          1000, 9910, members.data(), members.size()};
      EXPECT_TRUE(plain.InitializePhysical(
          {89,domain.node_count(),&group,1,{1000,.001}}));
      EXPECT_TRUE(rigid.Initialize(parts, &plain));
    } else {
      EXPECT_TRUE(rigid.Initialize(parts));
    }
    EXPECT_EQ(execution.Initialize(catalog, ledger, rigid).status,
        fe::ShellPlasticityBindingStatus::Success);
    const auto failures = source.Failures();
    EXPECT_EQ(failure.InitializeExecution(catalog, failures.data(),
        failures.size()).status, fe::ShellPlasticityBindingStatus::Success);
    EXPECT_TRUE(physical.InitializeExecution(
        {&shells,&catalog,&failure,nullptr}, ledger, execution));
    for (std::size_t row = 0; row < catalog.parent_count(); ++row) {
      const auto& parent = *catalog.parent(row);
      selection.push_back({row,parent.family,parent.family_index,
          parent.source_parent_id,parent.source_part_id});
    }
    EXPECT_EQ(surface.Initialize(physical,
        {selection.data(),selection.size()}).status,
        c::SelfContactSurfaceStatus::Ok);
    EXPECT_EQ(facets.Initialize(surface,{{},level}).status,
        c::FixedContactFacetStatus::Ok);
  }
  std::size_t Parent(std::uint64_t eid,
      const c::SelfContactActiveUseBinding& uses) const {
    for (std::size_t p = 0; p < uses.parents().size(); ++p)
      if (uses.parents()[p].source.source_parent_id == eid) return p;
    return SIZE_MAX;
  }
  std::size_t VertexUse(std::uint64_t eid,
      const c::SelfContactActiveUseBinding& uses,
      std::uint64_t source_nid = 0) const {
    const auto parent = Parent(eid, uses);
    for (std::size_t i = 0; i < uses.vertex_uses().size(); ++i) {
      const auto& use = uses.vertex_uses()[i];
      if (use.parent != parent) continue;
      if (!source_nid || (use.key.kind == c::FacetVertexKind::SourceVertex &&
          use.key.first == source_nid)) return i;
    }
    return SIZE_MAX;
  }
  std::size_t RemoteFacet(std::uint64_t eid, std::uint32_t vertex_feature,
      const c::SelfContactActiveUseBinding& uses) const {
    const auto parent = Parent(eid, uses);
    if (parent == SIZE_MAX) return SIZE_MAX;
    const auto& use = uses.parents()[parent];
    for (std::size_t i = use.facet_offset;
         i < std::size_t(use.facet_offset)+use.facet_count; ++i) {
      bool incident = false;
      for (const auto feature : uses.facet_uses()[i].vertex_features)
        incident = incident || feature == vertex_feature;
      if (!incident) return i;
    }
    return SIZE_MAX;
  }
  c::WeightedSurfacePoint FacePoint(std::size_t index,
      const c::SelfContactActiveUseBinding& uses) const {
    const auto& use = uses.facet_uses()[index];
    const auto& parent = uses.parents()[use.parent];
    c::FixedContactFacet facet;
    EXPECT_EQ(facets.Describe(parent.surface_parent,use.local_facet,&facet).status,
        c::FixedContactFacetStatus::Ok);
    constexpr std::array<double,3> barycentric{{1./3,1./3,1./3}};
    c::WeightedSurfacePoint point;
    EXPECT_EQ(c::ComposeFacetPoint(facet,barycentric.data(),
        domain.node_count(),&point),c::Status::kOk);
    return point;
  }
  std::vector<std::uint8_t> Active(
      const c::SelfContactActiveUseBinding& uses) const {
    return std::vector<std::uint8_t>(uses.parents().size(),1);
  }
};

struct Rigid {
  fe::rigid::NodalRigidPartTopology topology;
  fe::rigid::NodalRigidPartAssemblyModel parts;
  fe::NodalRigidGroupModel plain;
  fe::NodalRigidAssemblyBinding binding;

  Rigid(const Fixture& fixture,
      const std::vector<std::vector<std::uint64_t>>& part_nodes,
      const std::vector<std::uint64_t>& plain_nodes = {}) {
    std::vector<fe::rigid::PartTopologyPartInput> inputs;
    std::vector<std::uint64_t> expected;
    for (std::size_t p = 0; p < part_nodes.size(); ++p) {
      inputs.push_back({9000+p, part_nodes[p].data(), part_nodes[p].size()});
      expected.insert(expected.end(), part_nodes[p].begin(), part_nodes[p].end());
    }
    fe::rigid::PartTopologyInput input;
    input.source_instance_id = 77;
    input.parts = inputs.data();
    input.part_count = inputs.size();
    input.expected_members = expected.data();
    input.expected_member_count = expected.size();
    input.other_rigid_members = plain_nodes.data();
    input.other_rigid_member_count = plain_nodes.size();
    EXPECT_TRUE(topology.Initialize(input));
    EXPECT_TRUE(parts.Initialize(topology, fixture.ledger, {1000,.001}));
    if (plain_nodes.empty()) {
      EXPECT_TRUE(binding.Initialize(parts));
      return;
    }
    std::vector<fe::NodalRigidGroupMember> members;
    for (const auto id : plain_nodes) {
      const auto node = fixture.domain.Find(id);
      const auto& coefficient = fixture.ledger.nodes()[node].coefficients;
      members.push_back({id, node, fixture.domain.nodes()[node].position,
          coefficient.mass, coefficient.isotropic_inertia,
          coefficient.shell.physical_inertia, coefficient.shell.added_inertia, 0});
    }
    const fe::NodalRigidGroupInput group{9900,9910,members.data(),members.size()};
    EXPECT_TRUE(plain.InitializePhysical({89,fixture.domain.node_count(),&group,1,{1000,.001}}));
    EXPECT_TRUE(binding.Initialize(parts, &plain));
  }
};

struct Tie {
  std::array<std::int32_t,8192> decode{};
  tied::PostKinChkResult post;
  tied::TiedCinAttachmentModel model;
  std::array<cin::WitnessRange,1> ranges{{{0,1}}};
  std::array<cin::ActiveWitness,1> witnesses;
  explicit Tie(const Fixture& fixture) {
    tied::CinAttachmentDeclaration declaration;
    declaration.original_nsv_row = 3;
    declaration.ordered_master_rank = 19;
    declaration.master_source = {tied::CinMasterSourceKind::DeclaredShellElement,100,1000};
    declaration.topology = tied::CinMasterTopology::Quad;
    declaration.secondary_source_id = 20;
    declaration.reference_positions[0] =
        fixture.domain.nodes()[fixture.domain.Find(20)].position;
    for (unsigned i = 0; i < 4; ++i) {
      declaration.master_source_ids[i] = 10+i;
      declaration.reference_positions[i+1] =
          fixture.domain.nodes()[fixture.domain.Find(10+i)].position;
    }
    const tied::KinChkSlave slave{20,0,{2,7,7,0,0}};
    for (std::size_t i = 0; i < decode.size(); ++i) decode[i] = (i&2) != 0;
    const tied::KinChkInput kin{tied::KinChkProfile::NoWallRbeOrCyclic,
        tied::ClassificationPhase::InterfaceTaggedBeforeKinChk,77,881,
        {&slave,1},{decode.data(),decode.size()}};
    EXPECT_TRUE(tied::PostKinChk(kin, &post));
    EXPECT_TRUE(tied::PrepareCinAttachments(post, fixture.domain,
        {&declaration,1}, &model));
    auto& witness = witnesses[0];
    witness.source_element_id = 100;
    witness.native_parent_index = 0;
    witness.family = cin::WitnessFamily::ShellQuad;
    for (unsigned i = 0; i < 4; ++i)
      witness.nodes[i] = fixture.domain.Find(10+i);
  }
  c::SelfContactCinWitnessSource Source() const {
    return {&model,ranges.data(),witnesses.data(),ranges.size(),witnesses.size()};
  }
};
} // namespace active_use_test
