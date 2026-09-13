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
