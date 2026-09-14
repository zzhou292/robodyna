// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>
#include <limits>
#include <new>

namespace tlfea::contact {
namespace {

namespace activity = self_contact_physical_activity;
namespace fe = tl::fea;
using S = SelfContactPhysicalActivityStatus;

SelfContactPhysicalActivityReport Failure(
    S status, const char* message) noexcept {
  SelfContactPhysicalActivityReport report;
  report.status = status;
  report.message = message;
  return report;
}

SelfContactPhysicalActivityReport PublicationFailure(
    const fe::ShellPublicationReport& source,
    S status = S::PublicationFailure) noexcept {
  auto report = Failure(status, source.message);
  report.publication_status = source.status;
  report.owner_status = source.nodal_status;
  return report;
}

SelfContactPhysicalActivityReport OwnerFailure(
    const fe::NodalReport& source) noexcept {
  auto report = Failure(S::OwnerFailure, source.message);
  report.owner_status = source.status;
  return report;
}

template<class Report>
SelfContactPhysicalActivityReport FamilyFailure(
    fe::ShellBindingFamily family, const Report& source,
    S status) noexcept {
  auto report = Failure(status, source.message);
  report.family = family;
  report.family_index = source.element;
  report.owner_status = source.nodal_status;
  return report;
}

fe::ShellFormulationParticipants Shells(
    const fe::ShellPhysicalParticipants& participants) noexcept {
  return {participants.qeph, participants.t3, participants.qbat,
          participants.type25};
}

std::uint64_t SourceParent(
    const fe::ShellBatchBinding& shells,
    fe::ShellBindingFamily family, std::size_t index) noexcept {
  switch (family) {
    case fe::ShellBindingFamily::Qeph:
      return index < shells.qeph_count()
          ? shells.qeph_source_id(index) : 0;
    case fe::ShellBindingFamily::T3:
      return index < shells.t3_count()
          ? shells.t3_source_id(index) : 0;
    case fe::ShellBindingFamily::Qbat:
      return index < shells.qbat_count()
          ? shells.qbat_source_id(index) : 0;
    default:
      return 0;
  }
}

std::size_t FamilyCount(
    const fe::ShellBatchBinding& shells,
    fe::ShellBindingFamily family) noexcept {
  switch (family) {
    case fe::ShellBindingFamily::Qeph:
      return shells.qeph_count();
    case fe::ShellBindingFamily::T3:
      return shells.t3_count();
    case fe::ShellBindingFamily::Qbat:
      return shells.qbat_count();
    default:
      return 0;
  }
}

SelfContactPhysicalActivityReport ValidateSelection(
    const SelfContactActiveUseBinding& active_use,
    const fe::ShellPhysicalBinding& physical) noexcept {
  if (!active_use.prepared() || !active_use.identity() ||
      !active_use.facets() || !active_use.facets()->surface() ||
      !active_use.facets()->surface()->MatchesPhysical(physical) ||
      !physical.prepared() || !physical.shells() ||
      !physical.failure() || !physical.catalog())
    return Failure(S::IdentityMismatch,
        "Active-use and failure-capable physical source differ");
  const auto& shells = *physical.shells();
  const auto parents = active_use.parents();
  if (!parents.size())
    return Failure(S::InvalidInput,
        "Self-contact activity selection is empty");
  for (std::size_t parent = 0; parent < parents.size(); ++parent) {
    const auto& source = parents[parent].source;
    const auto count = FamilyCount(shells, source.family);
    if ((source.family != fe::ShellBindingFamily::Qeph &&
         source.family != fe::ShellBindingFamily::T3 &&
         source.family != fe::ShellBindingFamily::Qbat) ||
        source.family_index >= count ||
        !source.source_parent_id ||
        SourceParent(shells, source.family, source.family_index) !=
            source.source_parent_id) {
      auto report = Failure(S::IdentityMismatch,
          "Selected parent is not an exact QEPH/T3/QBAT inventory row");
      report.parent = parent;
      report.family = source.family;
      report.family_index = source.family_index;
      return report;
    }
    for (std::size_t prior = 0; prior < parent; ++prior) {
      const auto& other = parents[prior].source;
      if (other.family == source.family &&
          other.family_index == source.family_index) {
        auto report = Failure(S::IdentityMismatch,
            "Selected activity inventory repeats a physical family row");
        report.parent = parent;
        report.family = source.family;
        report.family_index = source.family_index;
        return report;
      }
    }
  }
  return {};
}

const std::uint8_t* FamilyValues(
    const activity::State& state,
    const fe::ShellPlasticityParentInput& source) noexcept {
  switch (source.family) {
    case fe::ShellBindingFamily::Qeph:
      return state.buffers.qeph;
    case fe::ShellBindingFamily::T3:
      return state.buffers.t3;
    case fe::ShellBindingFamily::Qbat:
      return state.buffers.qbat;
    default:
      return nullptr;
  }
}

SelfContactPhysicalActivityReport ValidateFamily(
    const std::uint8_t* values, std::size_t count,
    fe::ShellBindingFamily family) noexcept {
  for (std::size_t parent = 0; parent < count; ++parent) {
    if (values[parent] > 1) {
      auto report = Failure(S::InvalidActivity,
          "Actual complete-family activity readback is not binary");
      report.family = family;
      report.family_index = parent;
      return report;
    }
  }
  return {};
}

SelfContactPhysicalActivityReport ReadAcceptedFamilies(
    activity::State& state) noexcept {
  const auto& shells = *state.physical.shells();
  const auto stamp = state.owner->accepted();
  if (shells.qeph_count()) {
    fe::qeph::BatchDiagnostics diagnostics;
    const auto copied = state.participants.qeph->
        CopyAcceptedParentActivity(
            stamp, state.buffers.qeph, shells.qeph_count(),
            &diagnostics);
    if (copied.status != fe::qeph::BatchStatus::Success)
      return FamilyFailure(fe::ShellBindingFamily::Qeph,
                           copied, S::QephFailure);
  }
  if (shells.t3_count()) {
    fe::t3::BatchDiagnostics diagnostics;
    const auto copied = state.participants.t3->
        CopyAcceptedParentActivity(
            stamp, state.buffers.t3, shells.t3_count(),
            &diagnostics);
    if (copied.status != fe::t3::BatchStatus::Success)
      return FamilyFailure(fe::ShellBindingFamily::T3,
                           copied, S::T3Failure);
  }
  if (shells.qbat_count()) {
    fe::qbat::BatchDiagnostics diagnostics;
    const auto copied = state.participants.qbat->
        CopyAcceptedParentActivity(
            stamp, state.buffers.qbat, shells.qbat_count(),
            &diagnostics);
    if (copied.status != fe::qbat::BatchStatus::Success)
      return FamilyFailure(fe::ShellBindingFamily::Qbat,
                           copied, S::QbatFailure);
  }
  auto checked = ValidateFamily(
      state.buffers.qeph, shells.qeph_count(),
      fe::ShellBindingFamily::Qeph);
  if (checked.status != S::Ok) return checked;
  checked = ValidateFamily(
      state.buffers.t3, shells.t3_count(),
      fe::ShellBindingFamily::T3);
  if (checked.status != S::Ok) return checked;
  return ValidateFamily(
      state.buffers.qbat, shells.qbat_count(),
      fe::ShellBindingFamily::Qbat);
}

SelfContactPhysicalActivityReport ReadPreparedFamilies(
    activity::State& state, const fe::NodalTrialToken& token,
    const fe::ShellPhysicalDiagnostics& diagnostics) noexcept {
  const auto& shells = *state.physical.shells();
  if (shells.qeph_count()) {
    const auto copied = state.participants.qeph->
        CopyPreparedParentActivity(
            *state.owner, token, diagnostics.qeph,
            state.buffers.qeph, shells.qeph_count());
    if (copied.status != fe::qeph::BatchStatus::Success)
      return FamilyFailure(fe::ShellBindingFamily::Qeph,
                           copied, S::QephFailure);
  }
  if (shells.t3_count()) {
    const auto copied = state.participants.t3->
        CopyPreparedParentActivity(
            *state.owner, token, diagnostics.t3,
            state.buffers.t3, shells.t3_count());
    if (copied.status != fe::t3::BatchStatus::Success)
      return FamilyFailure(fe::ShellBindingFamily::T3,
                           copied, S::T3Failure);
  }
  if (shells.qbat_count()) {
    const auto copied = state.participants.qbat->
        CopyPreparedParentActivity(
            *state.owner, token, diagnostics.qbat,
            state.buffers.qbat, shells.qbat_count());
    if (copied.status != fe::qbat::BatchStatus::Success)
      return FamilyFailure(fe::ShellBindingFamily::Qbat,
                           copied, S::QbatFailure);
  }
  auto checked = ValidateFamily(
      state.buffers.qeph, shells.qeph_count(),
      fe::ShellBindingFamily::Qeph);
  if (checked.status != S::Ok) return checked;
  checked = ValidateFamily(
      state.buffers.t3, shells.t3_count(),
      fe::ShellBindingFamily::T3);
  if (checked.status != S::Ok) return checked;
  return ValidateFamily(
      state.buffers.qbat, shells.qbat_count(),
      fe::ShellBindingFamily::Qbat);
}

SelfContactPhysicalActivityReport Fail(
    activity::State& state,
    SelfContactPhysicalActivityReport report) noexcept {
  state.Invalidate();
  return report;
}

}  // namespace

SelfContactPhysicalActivity::SelfContactPhysicalActivity() noexcept =
    default;
SelfContactPhysicalActivity::~SelfContactPhysicalActivity() = default;

bool activity::State::OutputDisjoint(
    const void* output, std::size_t bytes) const noexcept {
  using fe::trial_identity::Disjoint;
  return output && publication &&
      active_use.OutputDisjoint(output, bytes) &&
      publication->PhysicalOutputDisjoint(output, bytes) &&
      Disjoint(output, bytes, this, sizeof(*this)) &&
      Disjoint(output, bytes, arena.data(), arena.bytes());
}

bool activity::State::AdvanceGeneration() noexcept {
  if (phase == Phase::Exhausted ||
      generation == std::numeric_limits<std::uint64_t>::max()) {
    phase = Phase::Exhausted;
    owner_id = base_epoch = attempt = 0;
    return false;
  }
  ++generation;
  return true;
}

void activity::State::Invalidate() noexcept {
  if (!AdvanceGeneration()) return;
  owner_id = base_epoch = attempt = 0;
  phase = Phase::Idle;
}

bool activity::State::CurrentOwnerEndpoint() const noexcept {
  if (!owner) return false;
  const auto stamp = owner->accepted();
  return stamp.owner_id == owner_id && stamp.epoch == base_epoch;
}

bool activity::State::Authenticates(
    const SelfContactAcceptedActivityReceipt& receipt) const noexcept {
  return phase == Phase::Accepted &&
      receipt.buffer_identity_ == buffers.accepted &&
      receipt.generation_ == generation &&
      receipt.owner_id_ == owner_id &&
      receipt.base_epoch_ == base_epoch &&
      receipt.attempt_ == attempt &&
      CurrentOwnerEndpoint();
}

bool activity::State::Authenticates(
    const SelfContactPreparedActivityReceipt& receipt) const noexcept {
  return phase == Phase::Prepared &&
      receipt.base_buffer_identity_ == buffers.accepted &&
      receipt.current_buffer_identity_ == buffers.current &&
      receipt.generation_ == generation &&
      receipt.owner_id_ == owner_id &&
      receipt.base_epoch_ == base_epoch &&
      receipt.attempt_ == attempt &&
      CurrentOwnerEndpoint();
}

SelfContactPhysicalActivityReport
activity::State::RevalidateSources() const noexcept {
  const auto physical_source = publication->ValidatePhysicalSources(
      *owner, physical, participants, identity);
  if (physical_source.status !=
      fe::ShellPublicationStatus::Success)
    return PublicationFailure(physical_source);
  const auto activity_source =
      publication->ValidateAcceptedActivitySources(
          *owner, Shells(participants),
          physical.shells()->inventory());
  if (activity_source.status !=
      fe::ShellPublicationStatus::Success)
    return PublicationFailure(activity_source);
  return ValidateSelection(active_use, physical);
}

SelfContactPhysicalActivityPreflight
SelfContactPhysicalActivity::Forecast(
    const SelfContactActiveUseBinding& active_use,
    const fe::ShellPhysicalBinding& physical,
    SelfContactPhysicalActivityLimits limits) noexcept {
  SelfContactPhysicalActivityPreflight result;
  const auto selection = ValidateSelection(active_use, physical);
  if (selection.status != S::Ok) {
    result.report = selection;
    return result;
  }
  constexpr std::size_t MaximumProfileBytes = 512u << 20;
  const auto selected = active_use.parents().size();
  const auto& shells = *physical.shells();
  const auto q = shells.qeph_count();
  const auto t = shells.t3_count();
  const auto b = shells.qbat_count();
  if (!limits.max_selected_parents ||
      !limits.max_family_parents ||
      !limits.max_host_bytes ||
      !limits.max_startup_host_bytes ||
      limits.max_host_bytes > MaximumProfileBytes ||
      limits.max_startup_host_bytes > MaximumProfileBytes ||
      selected > limits.max_selected_parents ||
      q > limits.max_family_parents ||
      t > limits.max_family_parents ||
      b > limits.max_family_parents) {
    result.report = Failure(S::ResourceLimit,
        "Physical activity counts or host profile exceed fixed limits");
    return result;
  }
  activity::Layout layout;
  if (!activity::MakeLayout(
          selected, q, t, b, limits.max_host_bytes, layout)) {
    result.report = Failure(S::ResourceLimit,
        "Physical activity arena exceeds its fixed host limit");
    return result;
  }
  SelfContactPhysicalActivityForecast forecast;
  forecast.selected_parent_count = selected;
  forecast.qeph_parent_count = q;
  forecast.t3_parent_count = t;
  forecast.qbat_parent_count = b;
  forecast.arena_bytes = layout.bytes;
  if (layout.bytes >
      SIZE_MAX - sizeof(SelfContactPhysicalActivity) -
          sizeof(activity::State)) {
    result.report = Failure(S::ResourceLimit,
        "Physical activity host accounting overflowed");
    return result;
  }
  forecast.owned_host_bytes =
      sizeof(SelfContactPhysicalActivity) +
      sizeof(activity::State) + layout.bytes;
  forecast.startup_host_bytes = forecast.owned_host_bytes;
  forecast.host_allocations = 1;
  if (forecast.owned_host_bytes > limits.max_host_bytes ||
      forecast.startup_host_bytes >
          limits.max_startup_host_bytes) {
    result.report = Failure(S::ResourceLimit,
        "Physical activity complete host payload exceeds its cap");
    return result;
  }
  result.forecast = forecast;
  return result;
}

SelfContactPhysicalActivityReport
SelfContactPhysicalActivity::Initialize(
    const SelfContactActiveUseBinding& active_use,
    fe::FENodalState& owner,
    fe::ShellBatchPublication& publication,
    const fe::ShellPhysicalBinding& physical,
    const fe::ShellPhysicalParticipants& participants,
    const fe::ShellPhysicalPublicationIdentity& identity,
    SelfContactPhysicalActivityLimits limits) noexcept try {
  if (state_)
    return Failure(S::AlreadyInitialized,
        "Physical activity authority is immutable");
  const auto physical_source = publication.ValidatePhysicalSources(
      owner, physical, participants, identity);
  if (physical_source.status !=
      fe::ShellPublicationStatus::Success)
    return PublicationFailure(physical_source);
  if (!physical.shells())
    return Failure(S::IdentityMismatch,
        "Physical shell inventory is absent");
  const auto activity_source =
      publication.ValidateAcceptedActivitySources(
          owner, Shells(participants),
          physical.shells()->inventory());
  if (activity_source.status !=
      fe::ShellPublicationStatus::Success)
    return PublicationFailure(activity_source);
  const auto preflight = Forecast(active_use, physical, limits);
  if (preflight.report.status != S::Ok)
    return preflight.report;

  auto next = std::make_shared<activity::State>(
      active_use, physical);
  next->owner = &owner;
  next->publication = &publication;
  next->participants = participants;
  next->identity = identity;
  next->storage_forecast = preflight.forecast;
  if (!activity::MakeLayout(
          preflight.forecast.selected_parent_count,
          preflight.forecast.qeph_parent_count,
          preflight.forecast.t3_parent_count,
          preflight.forecast.qbat_parent_count,
          limits.max_host_bytes, next->layout) ||
      next->layout.bytes != preflight.forecast.arena_bytes ||
      !next->arena.Initialize(next->layout.bytes))
    return Failure(S::ResourceLimit,
        "Physical activity arena allocation failed");
  const auto construct = [&](const tl::util::ArenaRegion& region) {
    return !region.count ||
        next->arena.Construct<std::uint8_t>(region) != nullptr;
  };
  if (!construct(next->layout.accepted) ||
      !construct(next->layout.current) ||
      !construct(next->layout.qeph) ||
      !construct(next->layout.t3) ||
      !construct(next->layout.qbat))
    return Failure(S::ResourceLimit,
        "Physical activity typed arena construction failed");
  next->buffers = activity::Bind(
      next->arena.data(), next->layout);
  state_ = std::move(next);
  return {};
} catch (const std::bad_alloc&) {
  return Failure(S::ResourceLimit,
      "Physical activity startup allocation failed");
}

SelfContactPhysicalActivityReport
SelfContactPhysicalActivity::CaptureAccepted(
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::NodalAssemblyView& view,
    SelfContactAcceptedActivityReceipt* output) noexcept {
  if (!state_)
    return Failure(S::NotInitialized,
        "Physical activity authority is not initialized");
  auto& state = *state_;
  using fe::trial_identity::Disjoint;
  if (&owner != state.owner || !output ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output), &view, sizeof(view)) ||
      !owner.AssemblyRangeDisjoint(
          token, view, output, sizeof(*output)))
    return Fail(state, Failure(S::InvalidInput,
        "Accepted activity owner/view/output is invalid or aliases source"));
  if (state.phase == activity::Phase::Exhausted)
    return Failure(S::ResourceLimit,
        "Physical activity generation space is exhausted");
  if (state.phase != activity::Phase::Idle &&
      state.owner_id == view.owner_id &&
      state.base_epoch == view.accepted.base_epoch &&
      state.attempt == view.attempt)
    return Fail(state, Failure(S::StaleReceipt,
        "This activity attempt was already captured"));

  const auto assembled = state.publication->
      ValidatePhysicalAssembly(owner, token, view);
  if (assembled.status != fe::ShellPublicationStatus::Success)
    return Fail(state, PublicationFailure(
        assembled, S::AssemblyIncomplete));
  auto checked = state.RevalidateSources();
  if (checked.status != S::Ok)
    return Fail(state, checked);
  checked = ReadAcceptedFamilies(state);
  if (checked.status != S::Ok)
    return Fail(state, checked);

  const auto parents = state.active_use.parents();
  for (std::size_t parent = 0; parent < parents.size(); ++parent) {
    const auto& source = parents[parent].source;
    const auto* family = FamilyValues(state, source);
    if (!family || family[source.family_index] > 1) {
      auto report = Failure(S::InvalidActivity,
          "Selected accepted activity cannot be mapped exactly");
      report.parent = parent;
      report.family = source.family;
      report.family_index = source.family_index;
      return Fail(state, report);
    }
  }
  if (!state.AdvanceGeneration())
    return Failure(S::ResourceLimit,
        "Physical activity generation space is exhausted");
  for (std::size_t parent = 0; parent < parents.size(); ++parent) {
    const auto& source = parents[parent].source;
    const auto value =
        FamilyValues(state, source)[source.family_index];
    state.buffers.accepted[parent] = value;
    state.buffers.current[parent] = value;
  }
  state.owner_id = view.owner_id;
  state.base_epoch = view.accepted.base_epoch;
  state.attempt = view.attempt;
  state.phase = activity::Phase::Accepted;

  SelfContactAcceptedActivityReceipt next;
  next.state_ = state_;
  next.buffer_identity_ = state.buffers.accepted;
  next.generation_ = state.generation;
  next.owner_id_ = state.owner_id;
  next.base_epoch_ = state.base_epoch;
  next.attempt_ = state.attempt;
  *output = std::move(next);
  return {};
}

SelfContactPhysicalActivityReport
SelfContactPhysicalActivity::CapturePrepared(
    fe::FENodalState& owner,
    const fe::NodalTrialToken& token,
    const fe::ShellPhysicalDiagnostics& diagnostics,
    const fe::NodalPreparedView& prepared,
    const SelfContactAcceptedActivityReceipt& accepted,
    SelfContactPreparedActivityReceipt* output) noexcept {
  if (!state_)
    return Failure(S::NotInitialized,
        "Physical activity authority is not initialized");
  auto& state = *state_;
  using fe::trial_identity::Disjoint;
  const auto retained = accepted.state_.lock();
  if (&owner != state.owner || !output ||
      retained.get() != &state ||
      !state.Authenticates(accepted) ||
      !state.OutputDisjoint(output, sizeof(*output)) ||
      !Disjoint(output, sizeof(*output), this, sizeof(*this)) ||
      !Disjoint(output, sizeof(*output), &token, sizeof(token)) ||
      !Disjoint(output, sizeof(*output),
                &diagnostics, sizeof(diagnostics)) ||
      !Disjoint(output, sizeof(*output),
                &prepared, sizeof(prepared)) ||
      !Disjoint(output, sizeof(*output),
                &accepted, sizeof(accepted)))
    return Fail(state, Failure(S::StaleReceipt,
        "Prepared activity inputs or accepted authority are invalid"));

  const auto candidate = state.publication->
      ValidatePhysicalCandidate(
          owner, token, diagnostics, prepared);
  if (candidate.status != fe::ShellPublicationStatus::Success)
    return Fail(state, PublicationFailure(candidate));
  auto checked = state.RevalidateSources();
  if (checked.status != S::Ok)
    return Fail(state, checked);
  checked = ReadPreparedFamilies(state, token, diagnostics);
  if (checked.status != S::Ok)
    return Fail(state, checked);

  const auto parents = state.active_use.parents();
  for (std::size_t parent = 0; parent < parents.size(); ++parent) {
    const auto& source = parents[parent].source;
    const auto* family = FamilyValues(state, source);
    if (!family) {
      auto report = Failure(S::IdentityMismatch,
          "Prepared activity family map is absent");
      report.parent = parent;
      report.family = source.family;
      report.family_index = source.family_index;
      return Fail(state, report);
    }
    const std::uint8_t base = state.buffers.accepted[parent];
    const std::uint8_t current = family[source.family_index];
    const auto transition =
        activity::ValidateTransition(&base, &current, 1);
    if (transition.status != S::Ok) {
      auto report = transition;
      report.parent = parent;
      report.family = source.family;
      report.family_index = source.family_index;
      return Fail(state, report);
    }
  }
  if (!state.AdvanceGeneration())
    return Failure(S::ResourceLimit,
        "Physical activity generation space is exhausted");
  for (std::size_t parent = 0; parent < parents.size(); ++parent) {
    const auto& source = parents[parent].source;
    state.buffers.current[parent] =
        FamilyValues(state, source)[source.family_index];
  }
  state.phase = activity::Phase::Prepared;

  SelfContactPreparedActivityReceipt next;
  next.state_ = state_;
  next.base_buffer_identity_ = state.buffers.accepted;
  next.current_buffer_identity_ = state.buffers.current;
  next.generation_ = state.generation;
  next.owner_id_ = state.owner_id;
  next.base_epoch_ = state.base_epoch;
  next.attempt_ = state.attempt;
  *output = std::move(next);
  return {};
}

void SelfContactPhysicalActivity::DiscardTrial() noexcept {
  if (state_) state_->Invalidate();
}

SelfContactPhysicalActivityForecast
SelfContactPhysicalActivity::forecast() const noexcept {
  return state_ ? state_->storage_forecast :
      SelfContactPhysicalActivityForecast{};
}

SelfContactPhysicalActivityAllocationInfo
SelfContactPhysicalActivity::allocations() const noexcept {
  return state_
      ? SelfContactPhysicalActivityAllocationInfo{
            state_->arena.bytes(), 1}
      : SelfContactPhysicalActivityAllocationInfo{};
}

bool SelfContactAcceptedActivityReceipt::valid() const noexcept {
  return activity().base != nullptr;
}

SelfContactActivityView
SelfContactAcceptedActivityReceipt::activity() const noexcept {
  const auto state = state_.lock();
  if (!state || !state->Authenticates(*this)) return {};
  return {state->buffers.accepted, state->buffers.accepted,
          state->active_use.parents().size()};
}

bool SelfContactPreparedActivityReceipt::valid() const noexcept {
  return activity().base != nullptr;
}

SelfContactActivityView
SelfContactPreparedActivityReceipt::activity() const noexcept {
  const auto state = state_.lock();
  if (!state || !state->Authenticates(*this)) return {};
  return {state->buffers.accepted, state->buffers.current,
          state->active_use.parents().size()};
}

}  // namespace tlfea::contact
