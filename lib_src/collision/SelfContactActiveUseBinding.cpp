// SPDX-License-Identifier: AGPL-3.0-or-later
#include "self_contact_active_use/Storage.h"
#include "lib_src/solvers/NodalTrialIdentity.h"
#include <new>

namespace tlfea::contact {
namespace {
template<class T> tl::util::ConstView<T> View(const T* values, std::size_t count) noexcept {
  static const T empty{};
  return {values ? values : &empty, count};
}
}
SelfContactActiveUsePreflight SelfContactActiveUseBinding::Preflight(
    const FixedContactFacetBinding& facets, SelfContactActiveUseSource source,
    SelfContactActiveUseLimits limits) noexcept {
  active_use::Layout layout;
  const auto report = active_use::MakeLayout(facets, source, limits, sizeof(Impl), layout);
  return {report, layout.forecast};
}
SelfContactActiveUseReport SelfContactActiveUseBinding::Initialize(
    const FixedContactFacetBinding& facets, SelfContactActiveUseSource source,
    SelfContactActiveUseLimits limits) noexcept try {
  using S = SelfContactActiveUseStatus;
  if (impl_) return {S::AlreadyInitialized, SIZE_MAX, SIZE_MAX,
      "Active-use binding is immutable"};
  if (!facets.OutputDisjoint(this, sizeof(*this)))
    return {S::InvalidInput, SIZE_MAX, SIZE_MAX,
        "Destination aliases fixed-facet/S0 source or source is absent"};
  active_use::Layout layout;
  auto report = active_use::MakeLayout(facets, source, limits, sizeof(Impl), layout);
  if (report.status != S::Ok) return report;
  auto next = std::make_shared<Impl>(facets, source);
  if (!next->arena.Initialize(layout.forecast.arena_bytes))
    return {S::ResourceLimit, SIZE_MAX, SIZE_MAX, "Active-use arena allocation failed"};
  auto& inventory = next->inventory;
  inventory.parents = next->arena.Construct<SelfContactParentUse>(layout.parents);
  inventory.facets = next->arena.Construct<SelfContactFacetUse>(layout.facets);
  inventory.vertices = next->arena.Construct<SelfContactVertexFeature>(layout.vertices);
  inventory.edges = next->arena.Construct<SelfContactEdgeFeature>(layout.edges);
  inventory.vertex_uses = next->arena.Construct<SelfContactFacetVertexUse>(layout.vertex_uses);
  inventory.edge_uses = next->arena.Construct<SelfContactFacetEdgeUse>(layout.edge_uses);
  inventory.node_roles = next->arena.Construct<active_use::NodeRole>(layout.node_roles);
  inventory.cin_ranges =
      next->arena.Construct<tl::constraints::tied_shell::cin::WitnessRange>(layout.cin_ranges);
  inventory.cin_witnesses =
      next->arena.Construct<tl::constraints::tied_shell::cin::ActiveWitness>(layout.cin_witnesses);
  if (!inventory.parents || !inventory.facets || !inventory.vertices || !inventory.edges ||
      !inventory.vertex_uses || !inventory.edge_uses || !inventory.node_roles ||
      !inventory.cin_ranges || !inventory.cin_witnesses)
    return {S::ResourceLimit, SIZE_MAX, SIZE_MAX, "Active-use arena construction failed"};
  {
    tl::util::HostArena startup;
    if (!startup.Initialize(layout.startup_arena_bytes))
      return {S::ResourceLimit, SIZE_MAX, SIZE_MAX,
          "Active-use canonical-order index allocation failed"};
    active_use::BuildScratch scratch;
    scratch.vertex_order =
        startup.Construct<std::uint32_t>(layout.vertex_order);
    scratch.edge_order =
        startup.Construct<std::uint32_t>(layout.edge_order);
    if (!scratch.vertex_order || !scratch.edge_order)
      return {S::ResourceLimit, SIZE_MAX, SIZE_MAX,
          "Active-use canonical-order index construction failed"};
    report = active_use::Build(facets, source, layout, scratch, inventory);
    if (report.status != S::Ok) return report;
  } // Canonical-order indexes retire before immutable publication.
  if (layout.forecast.cin_rows)
    inventory.cin_rows = next->cin_model.rows().data;
  next->forecast = layout.forecast;
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {SelfContactActiveUseStatus::ResourceLimit, SIZE_MAX, SIZE_MAX,
      "Active-use startup allocation failed"};
}
const FixedContactFacetBinding* SelfContactActiveUseBinding::facets() const noexcept {
  return impl_ ? &impl_->facets : nullptr;
}
const tl::fea::NodalRigidAssemblyBinding* SelfContactActiveUseBinding::rigid() const noexcept {
  return impl_ && impl_->rigid.prepared() ? &impl_->rigid : nullptr;
}
SelfContactCinWitnessSource SelfContactActiveUseBinding::cin() const noexcept {
  if (!impl_ || !impl_->forecast.cin_rows) return {};
  return {&impl_->cin_model, impl_->inventory.cin_ranges,
      impl_->inventory.cin_witnesses, impl_->forecast.cin_rows,
      impl_->forecast.cin_witnesses};
}
SelfContactActiveUsePolicy SelfContactActiveUseBinding::policy() const noexcept {
  return SelfContactActiveUsePolicy::
      SymmetricDirectedVertexAndEdgePointDualReferenceV2;
}
SelfContactActiveUseForecast SelfContactActiveUseBinding::forecast() const noexcept {
  return impl_ ? impl_->forecast : SelfContactActiveUseForecast{};
}
tl::util::ConstView<SelfContactParentUse> SelfContactActiveUseBinding::parents() const noexcept {
  return View(impl_ ? impl_->inventory.parents : nullptr, impl_ ? impl_->forecast.parents : 0);
}
tl::util::ConstView<SelfContactFacetUse> SelfContactActiveUseBinding::facet_uses() const noexcept {
  return View(impl_ ? impl_->inventory.facets : nullptr, impl_ ? impl_->forecast.facets : 0);
}
tl::util::ConstView<SelfContactVertexFeature> SelfContactActiveUseBinding::vertices() const noexcept {
  return View(impl_ ? impl_->inventory.vertices : nullptr, impl_ ? impl_->forecast.vertices : 0);
}
tl::util::ConstView<SelfContactEdgeFeature> SelfContactActiveUseBinding::edges() const noexcept {
  return View(impl_ ? impl_->inventory.edges : nullptr, impl_ ? impl_->forecast.edges : 0);
}
tl::util::ConstView<SelfContactFacetVertexUse> SelfContactActiveUseBinding::vertex_uses() const noexcept {
  return View(impl_ ? impl_->inventory.vertex_uses : nullptr,
      impl_ ? impl_->forecast.vertex_uses : 0);
}
tl::util::ConstView<SelfContactFacetEdgeUse> SelfContactActiveUseBinding::edge_uses() const noexcept {
  return View(impl_ ? impl_->inventory.edge_uses : nullptr,
      impl_ ? impl_->forecast.edge_uses : 0);
}
bool SelfContactActiveUseBinding::SharesStorage(
    const SelfContactActiveUseBinding& other) const noexcept {
  return impl_ && impl_ == other.impl_;
}
const void* SelfContactActiveUseBinding::identity() const noexcept {
  return impl_.get();
}
bool SelfContactActiveUseBinding::OutputDisjoint(const void* output,
    std::size_t bytes) const noexcept {
  using tl::fea::trial_identity::Disjoint;
  if (!impl_ || !Disjoint(output, bytes, this, sizeof(*this)) ||
      !Disjoint(output, bytes, impl_.get(), sizeof(Impl)) ||
      !Disjoint(output, bytes, impl_->arena.data(), impl_->arena.bytes()) ||
      !impl_->facets.OutputDisjoint(output, bytes))
    return false;
  if (impl_->rigid.prepared() &&
      (!Disjoint(output, bytes, impl_->rigid.groups().data(),
          impl_->rigid.groups().size()*sizeof(tl::fea::RigidBindingGroup)) ||
       !Disjoint(output, bytes, impl_->rigid.members().data(),
          impl_->rigid.members().size()*sizeof(tl::fea::RigidBindingMember))))
    return false;
  if (impl_->cin_model.prepared() &&
      !Disjoint(output, bytes, impl_->cin_model.rows().data,
          impl_->cin_model.rows().count*
          sizeof(tl::constraints::tied_shell::CinAttachmentRow)))
    return false;
  return true;
}
bool SelfContactActiveUseBinding::Authenticates(
    const SelfContactPairClassification& pair) const noexcept {
  return impl_ && pair.binding_identity == impl_.get();
}
SelfContactActiveUseReport SelfContactActiveUseBinding::ClassifySupport(
    const WeightedSurfacePoint& point,
    SelfContactSupportClassification* output) const noexcept {
  using S = SelfContactActiveUseStatus;
  if (!output || !OutputDisjoint(output, sizeof(*output)) ||
      !tl::fea::trial_identity::Disjoint(output, sizeof(*output), &point, sizeof(point)))
    return {S::InvalidInput, SIZE_MAX, SIZE_MAX, "Support output aliases input/source or binding is absent"};
  SelfContactSupportClassification next;
  const auto report = active_use::Classify(impl_->inventory, impl_->forecast, point, next);
  if (report.status == S::Ok) *output = next;
  return report;
}
} // namespace tlfea::contact
