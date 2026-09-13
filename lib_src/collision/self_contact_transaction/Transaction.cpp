// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>
#include <new>

namespace tlfea::contact {
namespace {

using S = SelfContactTransactionStatus;
namespace fe = tl::fea;
namespace sct = self_contact_transaction;

SelfContactTransactionReport Failure(S status,
                                     const char* message) noexcept {
  SelfContactTransactionReport result;
  result.status = status;
  result.message = message;
  return result;
}

bool ValidRange(const void* pointer, std::size_t count,
                std::size_t width) noexcept {
  if (!count) return pointer == nullptr;
  if (!pointer || count > SIZE_MAX / width) return false;
  const auto address = reinterpret_cast<std::uintptr_t>(pointer);
  return count * width <= UINTPTR_MAX - address;
}

bool EventLess(const sct::AcceptedEventIdentity& a,
               const sct::AcceptedEventIdentity& b) noexcept {
  const int feature =
      fixed_triangle_features::Compare(a.feature, b.feature);
  return feature < 0 ||
      (!feature && a.source_order < b.source_order);
}

}  // namespace

SelfContactTransaction::SelfContactTransaction() noexcept = default;
SelfContactTransaction::~SelfContactTransaction() = default;

bool SelfContactTransaction::Impl::OutputDisjoint(
    const void* output, std::size_t bytes) const noexcept {
  using fe::trial_identity::Disjoint;
  return output && active_use.OutputDisjoint(output, bytes) &&
      Disjoint(output, bytes, this, sizeof(*this)) &&
      Disjoint(output, bytes, arena.data(), arena.bytes()) &&
      publication && publication->PhysicalOutputDisjoint(output, bytes);
}

void SelfContactTransaction::Impl::DiscardLocal() noexcept {
  force.DiscardTrial();
  participation.DiscardTrial();
  activity_base_identity = nullptr;
  activity_current_identity = nullptr;
  accepted_event_count = 0;
  owner_id = base_epoch = attempt = 0;
  stream = nullptr;
  phase = Phase::Idle;
}

SelfContactTransactionReport SelfContactTransaction::Impl::Fail(
    SelfContactTransactionReport report) noexcept {
  if (owner) owner->Discard();
  if (publication) publication->DiscardTrial();
  DiscardLocal();
  return report;
}

SelfContactTransactionReport SelfContactTransaction::Initialize(
    const SelfContactTransactionConfig& config,
    const SelfContactActiveUseBinding& active_use,
    fe::FENodalState& owner,
    fe::ShellBatchPublication& publication,
    const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,
    const fe::ShellPhysicalPublicationIdentity& identity,
    SelfContactTransactionLimits limits) try {
  if (impl_)
    return Failure(S::AlreadyInitialized,
        "Self-contact transaction is immutable");
  if (!active_use.OutputDisjoint(this, sizeof(*this)) ||
      !fe::trial_identity::Disjoint(
          this, sizeof(*this), &config, sizeof(config)) ||
      !active_use.facets() || !active_use.facets()->surface() ||
      !active_use.facets()->surface()->MatchesPhysical(physical) ||
      !fe::trial_identity::SameStamp(
          owner.accepted(), config.force.owner))
    return Failure(S::IdentityMismatch,
        "Transaction active-use, physical or fresh owner identity differs");

  const auto source = publication.ValidatePhysicalSources(
      owner, physical, participants, identity);
  if (source.status != fe::ShellPublicationStatus::Success) {
    auto report = Failure(S::PublicationFailure, source.message);
    report.publication_status = source.status;
    report.owner_status = source.nodal_status;
    return report;
  }
  const auto preflight = Forecast(config, active_use, identity, limits);
  if (preflight.report.status != S::Ok) return preflight.report;

  auto next = std::make_unique<Impl>(active_use);
  next->owner = &owner;
  next->publication = &publication;
  next->config = config;
  next->storage_forecast = preflight.forecast;
  sct::Layout layout;
  const auto nodes = physical.domain()->node_count();
  if (!sct::MakeLayout(
          nodes, active_use.parents().size(),
          limits.max_candidate_triangles, limits.max_candidate_pairs,
          config.force.event_capacity, limits.max_host_bytes, layout) ||
      layout.bytes != preflight.forecast.candidate_arena_bytes ||
      !next->arena.Initialize(layout.bytes))
    return Failure(S::ResourceLimit,
        "Transaction candidate arena allocation failed");
  next->layout = layout;
  if (!next->arena.Construct<double>(layout.accepted_positions) ||
      !next->arena.Construct<double>(layout.accepted_velocities) ||
      !next->arena.Construct<double>(layout.prepared_positions) ||
      !next->arena.Construct<double>(layout.prepared_velocities) ||
      !next->arena.Construct<std::uint8_t>(layout.activity) ||
      !next->arena.Construct<CurrentFixedTriangle>(
          layout.current_triangles) ||
      !next->arena.Construct<RepresentedTrianglePath>(
          layout.represented_paths) ||
      !next->arena.Construct<RepresentedTrianglePair>(
          layout.represented_pairs) ||
      !next->arena.Construct<RepresentedIntervalPairKey>(
          layout.canonical_pairs) ||
      !next->arena.Construct<sct::AcceptedEventIdentity>(
          layout.accepted_events))
    return Failure(S::ResourceLimit,
        "Transaction typed candidate arena construction failed");
  next->buffers = sct::Bind(next->arena.data(), layout);

  const auto force =
      next->force.Initialize(config.force, active_use, owner, limits.force);
  if (force.status != SelfContactForceStatus::Ok) {
    auto report = Failure(S::ForceFailure, force.message);
    report.force_status = force.status;
    report.owner_status = force.owner_status;
    return report;
  }
  impl_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return Failure(S::ResourceLimit,
      "Self-contact transaction startup allocation failed");
}

fe::ShellPhysicalScratchRosterEntry
SelfContactTransaction::roster_entry() noexcept {
  if (!impl_) return {};
  return {&impl_->participation, impl_->config.source_id};
}

SelfContactTransactionReport SelfContactTransaction::AssembleAccepted(
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::NodalAssemblyView& view,
    SelfContactActivityView activity,
    SelfContactForceEventView events,
    SelfContactAcceptedAssemblyReceipt* output) {
  if (!impl_)
    return Failure(S::NotInitialized,
        "Self-contact transaction is not initialized");
  auto& state = *impl_;
  using fe::trial_identity::Disjoint;
  const auto parents = state.active_use.parents().size();
  const std::size_t activity_bytes = parents;
  const std::size_t event_bytes =
      events.count <= SIZE_MAX / sizeof(SelfContactForceEvent)
      ? events.count * sizeof(SelfContactForceEvent) : SIZE_MAX;
  if (&owner != state.owner || !output ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &view, sizeof(view)) ||
      activity.parent_count != parents ||
      !ValidRange(activity.base, parents, sizeof(std::uint8_t)) ||
      !ValidRange(activity.current, parents, sizeof(std::uint8_t)) ||
      !ValidRange(events.data, events.count,
                  sizeof(SelfContactForceEvent)) ||
      (activity.base != activity.current &&
       !Disjoint(activity.base, activity_bytes,
                 activity.current, activity_bytes)) ||
      !Disjoint(output, sizeof(*output),
                activity.base, activity_bytes) ||
      !Disjoint(output, sizeof(*output),
                activity.current, activity_bytes) ||
      !Disjoint(activity.base, activity_bytes,
                state.arena.data(), state.arena.bytes()) ||
      !Disjoint(activity.current, activity_bytes,
                state.arena.data(), state.arena.bytes()) ||
      (events.count &&
       (!Disjoint(output, sizeof(*output), events.data, event_bytes) ||
        !Disjoint(events.data, event_bytes,
                  state.arena.data(), state.arena.bytes()))))
    return state.Fail(Failure(S::InvalidInput,
        "Accepted transaction owner, activity, events or output are invalid"));

  if (state.phase != Impl::Phase::Idle) {
    if (view.attempt != state.attempt &&
        (owner.accepted().epoch == state.base_epoch ||
         owner.accepted().epoch == state.base_epoch + 1)) {
      state.DiscardLocal();
    } else {
      return state.Fail(Failure(S::PublicationFailure,
          "Self-contact assembly was already recorded for this attempt"));
    }
  }
  for (std::size_t parent = 0; parent < parents; ++parent) {
    if (activity.base[parent] != 1 || activity.current[parent] != 1)
      return state.Fail(Failure(S::UnsupportedActivity,
          "This safe slice requires every self-contact parent active"));
  }

  SelfContactForceAssemblyReceipt force_receipt;
  const auto force = state.force.AssembleAccepted(
      owner, token, view, events, &force_receipt);
  if (force.status != SelfContactForceStatus::Ok) {
    auto report = Failure(S::ForceFailure, force.message);
    report.force_status = force.status;
    report.owner_status = force.owner_status;
    return state.Fail(report);
  }
  const auto recorded =
      state.participation.RecordSelfContactAcceptedAssembly(
      state.config.source_id, owner, token, view);
  if (recorded.status != fe::ShellPublicationStatus::Success) {
    auto report = Failure(S::PublicationFailure, recorded.message);
    report.publication_status = recorded.status;
    report.owner_status = recorded.nodal_status;
    return state.Fail(report);
  }

  std::copy_n(activity.base, parents, state.buffers.activity);
  for (std::size_t i = 0; i < events.count; ++i)
    state.buffers.accepted_events[i] =
        {events.data[i].feature, events.data[i].source_order};
  std::sort(state.buffers.accepted_events,
            state.buffers.accepted_events + events.count, EventLess);
  state.activity_base_identity = activity.base;
  state.activity_current_identity = activity.current;
  state.accepted_event_count = events.count;
  state.owner_id = view.owner_id;
  state.base_epoch = view.accepted.base_epoch;
  state.attempt = view.attempt;
  state.stream = view.stream;
  state.phase = Impl::Phase::AssemblyRecorded;

  SelfContactAcceptedAssemblyReceipt next;
  next.transaction_ = this;
  next.owner_ = &owner;
  next.active_use_identity_ = state.active_use.identity();
  next.source_id_ = state.config.source_id;
  next.configuration_id_ = state.config.force.configuration_id;
  next.qualification_id_ = state.config.force.qualification_id;
  next.owner_id_ = state.owner_id;
  next.base_epoch_ = state.base_epoch;
  next.attempt_ = state.attempt;
  next.force_ = force_receipt;
  *output = next;
  return {};
}

void SelfContactTransaction::DiscardTrial() noexcept {
  if (impl_) impl_->DiscardLocal();
}

SelfContactTransactionForecast SelfContactTransaction::forecast()
    const noexcept {
  return impl_ ? impl_->storage_forecast :
      SelfContactTransactionForecast{};
}

fe::NodalAllocationInfo SelfContactTransaction::allocations()
    const noexcept {
  return impl_ ? impl_->force.allocations() :
      fe::NodalAllocationInfo{};
}

}  // namespace tlfea::contact
