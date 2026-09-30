// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../SelfContactSurfaceBinding.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::self_contact {
struct Layout {
  tl::util::ArenaRegion parents, vertices, edges, vertex_uses, edge_uses, faces;
  SelfContactSurfaceForecast forecast;
};
SelfContactSurfaceReport MakeLayout(const tl::fea::ShellPhysicalBinding&,
    const SelfContactSurfaceInput&, SelfContactSurfaceLimits, std::size_t, Layout&) noexcept;
SelfContactSurfaceReport ValidateSources(const tl::fea::ShellPhysicalBinding&,
    const SelfContactSurfaceInput&);
SelfContactSurfaceReport ReadParent(const tl::fea::ShellPhysicalBinding&,
    const SelfContactParentSelection&, std::size_t, SelfContactSurfaceParent&) noexcept;
struct Features {
  SelfContactSurfaceParent* parents = nullptr;
  SelfContactVertex* vertices = nullptr;
  SelfContactEdge* edges = nullptr;
  SelfContactVertexUse* vertex_uses = nullptr;
  SelfContactEdgeUse* edge_uses = nullptr;
  std::uint32_t* faces = nullptr;
  std::size_t vertex_count = 0, edge_count = 0, use_count = 0;
};
SelfContactSurfaceReport BuildFeatures(const tl::fea::ShellPhysicalBinding&,
    const SelfContactSurfaceForecast&, Features&) noexcept;
} // namespace tlfea::contact::self_contact

namespace tlfea::contact {
struct SelfContactSurfaceBinding::Impl {
  explicit Impl(const tl::fea::ShellPhysicalBinding& source) : physical(source) {}
  tl::fea::ShellPhysicalBinding physical;
  tl::util::HostArena arena;
  self_contact::Features features;
  SelfContactSurfaceForecast forecast;
};
} // namespace tlfea::contact
