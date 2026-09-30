// SPDX-License-Identifier: AGPL-3.0-or-later
#include "self_contact/Storage.h"
#include "../elements/ShellPhysicalOutputRanges.h"
#include "../solvers/NodalTrialIdentity.h"
#include <new>

namespace tlfea::contact {
SelfContactSurfacePreflight SelfContactSurfaceBinding::Preflight(const tl::fea::ShellPhysicalBinding& physical,
    const SelfContactSurfaceInput& input, SelfContactSurfaceLimits limits) noexcept try {
  self_contact::Layout layout;
  auto report = self_contact::MakeLayout(physical, input, limits, sizeof(Impl), layout);
  if (report.status != SelfContactSurfaceStatus::Ok) return {report, {}};
  report = self_contact::ValidateSources(physical, input);
  return {report, layout.forecast};
} catch (const std::bad_alloc&) {
  return {{SelfContactSurfaceStatus::ResourceLimit, SIZE_MAX, "Surface validation index allocation failed"}, {}};
}
SelfContactSurfaceReport SelfContactSurfaceBinding::Initialize(const tl::fea::ShellPhysicalBinding& physical,
    const SelfContactSurfaceInput& input, SelfContactSurfaceLimits limits) noexcept try {
  using S = SelfContactSurfaceStatus;
  if (impl_) return {S::AlreadyInitialized, SIZE_MAX, "Surface binding is immutable"};
  if (!tl::fea::shell_physical_owner::OutputDisjoint(physical, this, sizeof(*this)))
    return {S::InvalidInput, SIZE_MAX, "Destination aliases retained physical source"};
  self_contact::Layout layout;
  auto report = self_contact::MakeLayout(physical, input, limits, sizeof(Impl), layout);
  if (report.status != S::Ok) return report;
  if (!tl::fea::trial_identity::Disjoint(this, sizeof(*this), input.parents,
      input.parent_count * sizeof(SelfContactParentSelection)))
    return {S::InvalidInput, SIZE_MAX, "Destination aliases borrowed selection"};
  report = self_contact::ValidateSources(physical, input);
  if (report.status != S::Ok) return report;
  // Validation index has retired. All following writes target unpublished storage.
  auto next = std::make_shared<Impl>(physical);
  if (!next->arena.Initialize(layout.forecast.arena_bytes))
    return {S::ResourceLimit, SIZE_MAX, "Surface arena allocation failed"};
  auto& f = next->features;
  f.parents = next->arena.Construct<SelfContactSurfaceParent>(layout.parents);
  f.vertices = next->arena.Construct<SelfContactVertex>(layout.vertices);
  f.edges = next->arena.Construct<SelfContactEdge>(layout.edges);
  f.vertex_uses = next->arena.Construct<SelfContactVertexUse>(layout.vertex_uses);
  f.edge_uses = next->arena.Construct<SelfContactEdgeUse>(layout.edge_uses);
  f.faces = next->arena.Construct<std::uint32_t>(layout.faces);
  if (!f.parents || !f.vertices || !f.edges || !f.vertex_uses || !f.edge_uses || !f.faces)
    return {S::ResourceLimit, SIZE_MAX, "Surface typed arena layout rejected"};
  for (std::size_t p = 0; p < input.parent_count; ++p) {
    report = self_contact::ReadParent(physical, input.parents[p], p, f.parents[p]);
    if (report.status != S::Ok) return report;
  }
  report = self_contact::BuildFeatures(physical, layout.forecast, f);
  if (report.status != S::Ok) return report;
  next->forecast = layout.forecast;
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return {SelfContactSurfaceStatus::ResourceLimit, SIZE_MAX, "Surface allocation failed"};
}
const tl::fea::ShellPhysicalBinding* SelfContactSurfaceBinding::physical() const noexcept {
  return impl_ ? &impl_->physical : nullptr;
}
SelfContactSurfaceProfile SelfContactSurfaceBinding::profile() const noexcept {
  return SelfContactSurfaceProfile::FrictionlessReferenceThicknessShellSubsetV1;
}
SelfContactSurfaceForecast SelfContactSurfaceBinding::forecast() const noexcept {
  return impl_ ? impl_->forecast : SelfContactSurfaceForecast{};
}
bool SelfContactSurfaceBinding::SharesStorage(const SelfContactSurfaceBinding& other) const noexcept {
  return impl_ && impl_ == other.impl_;
}
bool SelfContactSurfaceBinding::MatchesPhysical(const tl::fea::ShellPhysicalBinding& physical) const noexcept {
  return impl_ && impl_->physical.Matches(physical);
}
bool SelfContactSurfaceBinding::OutputDisjoint(const void* output, std::size_t bytes) const noexcept {
  return impl_ && tl::fea::trial_identity::Disjoint(output, bytes, this, sizeof(*this)) &&
      tl::fea::trial_identity::Disjoint(output, bytes, impl_.get(), sizeof(Impl)) &&
      tl::fea::trial_identity::Disjoint(output, bytes, impl_->arena.data(), impl_->arena.bytes()) &&
      tl::fea::shell_physical_owner::OutputDisjoint(impl_->physical, output, bytes);
}
} // namespace tlfea::contact
