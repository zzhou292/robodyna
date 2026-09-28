// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <new>
namespace tl::fea::physical_activity {
State::~State() { if (arena) cudaFree(arena); }
}
namespace tl::fea {
PhysicalActivitySnapshot::PhysicalActivitySnapshot() noexcept = default;
PhysicalActivitySnapshot::~PhysicalActivitySnapshot() = default;
PhysicalActivityReport PhysicalActivitySnapshot::Initialize(FENodalState& owner,
    ShellBatchPublication& publication, const ShellPhysicalBinding& physical,
    const ShellPhysicalParticipants& participants, const ShellPhysicalPublicationIdentity& identity,
    PhysicalActivityLimits limits) noexcept try {
  namespace a = physical_activity; using S = PhysicalActivityStatus;
  if (state_) return {S::AlreadyInitialized, "Physical activity source is immutable"};
  if (owner.accepted().epoch) return {S::InvalidInput, "Initialize activity before the first interval"};
  const auto checked = publication.ValidatePhysicalSources(owner, physical, participants, identity);
  if (checked.status != ShellPublicationStatus::Success) return a::PublicationReport(checked);
  PhysicalActivityForecast forecast;
  auto report = Preflight(physical, limits, forecast);
  if (report.status != S::Ok) return report;
  auto next = std::make_shared<a::State>(physical);
  next->owner = &owner; next->publication = &publication; next->participants = participants;
  next->identity = identity; next->forecast = forecast;
  const auto stream = owner.BorrowOwnerStream(&next->stream);
  if (stream.status != NodalStatus::Ok) return {S::OwnerFailure, stream.message};
  const auto count = forecast.qeph_count + forecast.t3_count;
  if (!a::MakeLayout(count, limits.max_device_bytes, next->layout))
    return {S::ResourceLimit, "Physical activity layout exceeds its limit"};
  util::HostArena roles;
  if (count && !roles.Initialize(count)) return {S::ResourceLimit, "Activity source law staging failed"};
  auto* laws = static_cast<std::uint8_t*>(roles.data());
  for (unsigned family = 0; family < 2; ++family) {
    const auto tag = family ? ShellBindingFamily::T3 : ShellBindingFamily::Qeph;
    const auto offset = family ? forecast.qeph_count : 0;
    const auto n = family ? forecast.t3_count : forecast.qeph_count;
    for (std::size_t parent = 0; parent < n; ++parent) {
      ShellSectionLaw law = ShellSectionLaw::Unspecified;
      if (!physical.catalog()->Law(tag, parent, &law) ||
          (law != ShellSectionLaw::LayeredLaw1Nip3 && law != ShellSectionLaw::LayeredLaw44Nip3 &&
           !(tag == ShellBindingFamily::T3 && law == ShellSectionLaw::Law44Nip1) &&
           !(physical.catalog()->execution_sections() &&
             (law == ShellSectionLaw::RigidSkin || law == ShellSectionLaw::GlobalLaw1Npt0))))
        return {S::SourceMismatch, "Physical activity source section law is unsupported"};
      laws[offset + parent] = static_cast<std::uint8_t>(law);
    }
  }
  if (cudaMalloc(&next->arena, next->layout.bytes) != cudaSuccess)
    return {S::ResourceLimit, "Physical activity device arena allocation failed"};
  next->base = util::ArenaPointer<std::uint8_t>(next->arena, next->layout.masks[0]);
  next->current = util::ArenaPointer<std::uint8_t>(next->arena, next->layout.masks[1]);
  next->staging = util::ArenaPointer<std::uint8_t>(next->arena, next->layout.masks[2]);
  next->laws = util::ArenaPointer<std::uint8_t>(next->arena, next->layout.laws);
  next->device_control = util::ArenaPointer<a::FamilyControl>(next->arena, next->layout.control);
  if (count && (cudaMemcpyAsync(next->laws, laws, count, cudaMemcpyHostToDevice, next->stream) != cudaSuccess ||
                cudaStreamSynchronize(next->stream) != cudaSuccess))
    return {S::DeviceFailure, "Physical activity source law upload failed"};
  state_ = std::move(next); return {};
} catch (const std::bad_alloc&) {
  return {PhysicalActivityStatus::ResourceLimit, "Physical activity host allocation failed"};
}
PhysicalActivityForecast PhysicalActivitySnapshot::allocations() const noexcept {
  return state_ ? state_->forecast : PhysicalActivityForecast{};
}
void PhysicalActivitySnapshot::DiscardTrial() noexcept { if (state_) state_->Invalidate(); }
} // namespace tl::fea
