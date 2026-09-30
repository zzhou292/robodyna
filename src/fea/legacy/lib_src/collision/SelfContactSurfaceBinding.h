// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "SelfContactSurfaceTypes.h"
#include "../assembly/ShellPhysicalBinding.h"

namespace tlfea::contact {
// Immutable shell subset and feature incidence. Caller selection is copied;
// existing physical/catalog/reference backing is retained without recopying.
// This handle supplies no broadphase, contact force, activity, body/tie exclusion,
// mass response, owner clock or participant admission. Current coordinates are
// always a future caller's stamped owner view, never a stored reference proxy.
class SelfContactSurfaceBinding {
 public:
  SelfContactSurfaceBinding() = default;
  SelfContactSurfaceBinding(const SelfContactSurfaceBinding&) noexcept = default;
  SelfContactSurfaceBinding(SelfContactSurfaceBinding&& other) noexcept : impl_(other.impl_) {}
  SelfContactSurfaceBinding& operator=(const SelfContactSurfaceBinding&) = delete;
  static SelfContactSurfacePreflight Preflight(const tl::fea::ShellPhysicalBinding&,
      const SelfContactSurfaceInput&, SelfContactSurfaceLimits = {}) noexcept;
  SelfContactSurfaceReport Initialize(const tl::fea::ShellPhysicalBinding&,
      const SelfContactSurfaceInput&, SelfContactSurfaceLimits = {}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  const tl::fea::ShellPhysicalBinding* physical() const noexcept;
  SelfContactSurfaceProfile profile() const noexcept;
  SelfContactSurfaceForecast forecast() const noexcept;
  tl::util::ConstView<SelfContactSurfaceParent> parents() const noexcept;
  tl::util::ConstView<SelfContactVertex> vertices() const noexcept;
  tl::util::ConstView<SelfContactEdge> edges() const noexcept;
  tl::util::ConstView<SelfContactVertexUse> vertex_uses() const noexcept;
  tl::util::ConstView<SelfContactEdgeUse> edge_uses() const noexcept;
  // Parent indices sorted by original source EID. Parents themselves retain the
  // caller's selection order; equal geometry never merges source faces.
  tl::util::ConstView<std::uint32_t> faces() const noexcept;
  bool SharesStorage(const SelfContactSurfaceBinding&) const noexcept;
  bool MatchesPhysical(const tl::fea::ShellPhysicalBinding&) const noexcept;
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  TopologicalIncidence VertexInFace(std::size_t vertex, std::size_t parent) const noexcept;
  TopologicalIncidence EdgesShareVertex(std::size_t first, std::size_t second) const noexcept;
 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tlfea::contact
