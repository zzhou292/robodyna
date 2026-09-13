// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../SelfContactActiveUseBinding.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::active_use {
struct NodeRole {
  std::uint32_t rigid_group = UINT32_MAX;
  std::uint8_t cin_secondary = 0, cin_master = 0;
};
struct LocalVertex {
  FacetVertexKey key;
  WeightedSurfacePoint point;
  std::uint32_t valence = 0;
};
struct LocalEdge {
  FacetEdgeKey key;
  WeightedSurfacePoint endpoints[2];
  std::uint32_t valence = 0;
};
inline constexpr std::size_t MaxLocalVertices = 25, MaxLocalEdges = 56;
struct Layout {
  tl::util::ArenaRegion parents, facets, vertices, edges, vertex_uses, edge_uses;
  tl::util::ArenaRegion node_roles, cin_ranges, cin_witnesses;
  SelfContactActiveUseForecast forecast;
};
struct Inventory {
  SelfContactParentUse* parents = nullptr;
  SelfContactFacetUse* facets = nullptr;
  SelfContactVertexFeature* vertices = nullptr;
  SelfContactEdgeFeature* edges = nullptr;
  SelfContactFacetVertexUse* vertex_uses = nullptr;
  SelfContactFacetEdgeUse* edge_uses = nullptr;
  NodeRole* node_roles = nullptr;
  const tl::constraints::tied_shell::CinAttachmentRow* cin_rows = nullptr;
  tl::constraints::tied_shell::cin::WitnessRange* cin_ranges = nullptr;
  tl::constraints::tied_shell::cin::ActiveWitness* cin_witnesses = nullptr;
};
SelfContactActiveUseReport MakeLayout(const FixedContactFacetBinding&,
    SelfContactActiveUseSource, SelfContactActiveUseLimits, std::size_t,
    Layout&) noexcept;
SelfContactActiveUseReport Build(const FixedContactFacetBinding&,
    SelfContactActiveUseSource, const Layout&, Inventory&) noexcept;
SelfContactActiveUseReport Classify(const Inventory&,
    const SelfContactActiveUseForecast&, const WeightedSurfacePoint&,
    SelfContactSupportClassification&) noexcept;
bool ValidateActivity(const SelfContactActiveUseForecast&, SelfContactActivityView) noexcept;
bool MapMatchesParent(const SelfContactParentUse&, const WeightedSurfacePoint&) noexcept;
SelfContactTiedStatus TiedStatus(const Inventory&, const SelfContactActiveUseForecast&,
    const SelfContactParentUse&, const WeightedSurfacePoint&,
    const SelfContactParentUse&, const WeightedSurfacePoint&) noexcept;
} // namespace tlfea::contact::active_use

namespace tlfea::contact {
struct SelfContactActiveUseBinding::Impl {
  Impl(const FixedContactFacetBinding& source, SelfContactActiveUseSource support)
      : facets(source),
        rigid(support.rigid ? *support.rigid : tl::fea::NodalRigidAssemblyBinding{}),
        cin_model(support.cin.model ? *support.cin.model :
            tl::constraints::tied_shell::TiedCinAttachmentModel{}) {}
  FixedContactFacetBinding facets;
  tl::fea::NodalRigidAssemblyBinding rigid;
  tl::constraints::tied_shell::TiedCinAttachmentModel cin_model;
  tl::util::HostArena arena;
  active_use::Inventory inventory;
  SelfContactActiveUseForecast forecast;
};
} // namespace tlfea::contact
