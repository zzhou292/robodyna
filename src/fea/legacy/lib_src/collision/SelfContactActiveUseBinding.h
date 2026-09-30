// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "SelfContactActiveUseTypes.h"
#include "SelfContactActiveUseValues.h"
#include "FixedContactFacetBinding.h"

namespace tlfea::contact {
namespace self_contact_transaction {
struct ActiveUseQueryAccess;
}

// Immutable parent-local physical-feature uses. Canonical feature keys provide
// topology only: every parent use keeps its own thickness, certified reference
// area, weighted map and support classification. No geometry query, force,
// broadphase, crossing test, owner participation or state clock is added.
class SelfContactActiveUseBinding {
 public:
  SelfContactActiveUseBinding() = default;
  SelfContactActiveUseBinding(const SelfContactActiveUseBinding&) noexcept = default;
  SelfContactActiveUseBinding(SelfContactActiveUseBinding&& other) noexcept : impl_(other.impl_) {}
  SelfContactActiveUseBinding& operator=(const SelfContactActiveUseBinding&) = delete;
  static SelfContactActiveUsePreflight Preflight(const FixedContactFacetBinding&,
      SelfContactActiveUseSource = {}, SelfContactActiveUseLimits = {}) noexcept;
  SelfContactActiveUseReport Initialize(const FixedContactFacetBinding&,
      SelfContactActiveUseSource = {}, SelfContactActiveUseLimits = {}) noexcept;
  bool prepared() const noexcept { return bool(impl_); }
  const FixedContactFacetBinding* facets() const noexcept;
  const tl::fea::NodalRigidAssemblyBinding* rigid() const noexcept;
  SelfContactCinWitnessSource cin() const noexcept;
  SelfContactActiveUsePolicy policy() const noexcept;
  SelfContactActiveUseForecast forecast() const noexcept;
  tl::util::ConstView<SelfContactParentUse> parents() const noexcept;
  tl::util::ConstView<SelfContactFacetUse> facet_uses() const noexcept;
  tl::util::ConstView<SelfContactVertexFeature> vertices() const noexcept;
  tl::util::ConstView<SelfContactEdgeFeature> edges() const noexcept;
  tl::util::ConstView<SelfContactFacetVertexUse> vertex_uses() const noexcept;
  tl::util::ConstView<SelfContactFacetEdgeUse> edge_uses() const noexcept;
  bool SharesStorage(const SelfContactActiveUseBinding&) const noexcept;
  const void* identity() const noexcept;
  bool OutputDisjoint(const void*, std::size_t) const noexcept;
  bool Authenticates(const SelfContactPairClassification&) const noexcept;

  SelfContactActiveUseReport ClassifySupport(const WeightedSurfacePoint&,
      SelfContactSupportClassification*) const noexcept;
  SelfContactActiveUseReport ResolveVertexUse(std::size_t,
      SelfContactActivityView, SelfContactResolvedVertexUse*) const noexcept;
  SelfContactActiveUseReport ResolveEdgeUse(std::size_t,
      SelfContactActivityView, SelfContactResolvedEdgeUse*) const noexcept;
  SelfContactActiveUseReport ClassifyVertexFace(std::size_t vertex_use,
      std::size_t facet_use, const WeightedSurfacePoint& face_point,
      SelfContactActivityView, SelfContactPairClassification*) const noexcept;
  // Strict interior and zero-distance EE queries derive a symmetric force area
  // only from each exact edge use's certified endpoint directed dual areas.
  // Other EE geometry cases remain explicitly unadmitted.
  SelfContactActiveUseReport ClassifyEdgeEdge(std::size_t first_edge_use,
      const WeightedSurfacePoint& first_point, std::size_t second_edge_use,
      const WeightedSurfacePoint& second_point, SelfContactEdgeEdgeCase,
      SelfContactActivityView,
      SelfContactPairClassification*) const noexcept;
 private:
  friend struct self_contact_transaction::ActiveUseQueryAccess;
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};
} // namespace tlfea::contact
