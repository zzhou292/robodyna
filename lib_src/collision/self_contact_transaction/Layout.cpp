// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <limits>

namespace tlfea::contact {
namespace {

using S = SelfContactTransactionStatus;

SelfContactTransactionPreflight Failure(S status,
                                        const char* message) noexcept {
  SelfContactTransactionPreflight result;
  result.report.status = status;
  result.report.message = message;
  return result;
}

bool Add(std::size_t value, std::size_t* total) noexcept {
  if (!total || value > SIZE_MAX - *total) return false;
  *total += value;
  return true;
}

}  // namespace

SelfContactTransactionPreflight SelfContactTransaction::Forecast(
    const SelfContactTransactionConfig& config,
    const SelfContactActiveUseBinding& active_use,
    const tl::fea::ShellPhysicalPublicationIdentity& identity,
    SelfContactTransactionLimits limits) noexcept {
  namespace fe = tl::fea;
  namespace sct = self_contact_transaction;
  if (!active_use.prepared() || !active_use.identity() ||
      !active_use.facets() || !active_use.facets()->surface() ||
      !active_use.facets()->surface()->physical() ||
      !config.source_id || !identity.configuration_id ||
      !identity.qualification_id ||
      config.force.configuration_id != identity.configuration_id ||
      config.force.qualification_id != identity.qualification_id ||
      !fe::shell_startup_detail::SameStartup(
          config.force.startup, identity.startup) ||
      !limits.max_candidate_triangles || !limits.max_candidate_pairs ||
      !limits.max_host_bytes || !limits.max_device_bytes ||
      !limits.max_startup_host_bytes)
    return Failure(S::InvalidInput,
        "Transaction source, identities, startup or limits are invalid");

  const auto force = SelfContactForceAssembly::Forecast(
      config.force, active_use, limits.force);
  if (force.report.status != SelfContactForceStatus::Ok) {
    auto result = Failure(S::ForceFailure, force.report.message);
    result.report.force_status = force.report.status;
    return result;
  }

  fe::ShellPhysicalScratchParticipation dummy;
  const fe::ShellPhysicalScratchRoster roster{
      {}, {&dummy, config.source_id}};
  fe::ShellPhysicalScratchParticipationForecast participation;
  const auto participation_report =
      fe::ShellBatchPublication::ForecastPhysicalScratchParticipation(
          roster, limits.participation, participation);
  if (participation_report.status !=
      fe::ShellPublicationStatus::Success) {
    auto result = Failure(
        participation_report.status == fe::ShellPublicationStatus::ResourceLimit
            ? S::ResourceLimit : S::PublicationFailure,
        participation_report.message);
    result.report.publication_status = participation_report.status;
    return result;
  }

  const auto nodes =
      active_use.facets()->surface()->physical()->domain()->node_count();
  const auto parents = active_use.parents().size();
  sct::Layout layout;
  if (!sct::MakeLayout(
          nodes, parents, limits.max_candidate_triangles,
          limits.max_candidate_pairs, config.force.event_capacity,
          limits.max_host_bytes, layout))
    return Failure(S::ResourceLimit,
        "Transaction candidate arena exceeds its fixed profile");

  SelfContactTransactionForecast forecast;
  forecast.force = force.forecast;
  forecast.participation = participation;
  forecast.parent_activity_bytes = parents;
  if (nodes > SIZE_MAX / 6)
    return Failure(S::ResourceLimit,
        "Transaction snapshot value count overflowed");
  forecast.accepted_snapshot_values = 6 * nodes;
  forecast.prepared_snapshot_values = 6 * nodes;
  forecast.candidate_triangle_capacity =
      limits.max_candidate_triangles;
  forecast.candidate_pair_capacity = limits.max_candidate_pairs;
  forecast.accepted_event_identity_capacity =
      config.force.event_capacity;
  forecast.candidate_arena_bytes = layout.bytes;
  forecast.device_bytes = force.forecast.device_bytes;
  forecast.device_allocations = force.forecast.device_allocations;
  if (forecast.device_bytes > limits.max_device_bytes)
    return Failure(S::ResourceLimit,
        "Transaction device payload exceeds its complete cap");

  if (force.forecast.owned_host_bytes <
      sizeof(SelfContactForceAssembly) ||
      force.forecast.startup_host_bytes <
      sizeof(SelfContactForceAssembly))
    return Failure(S::ResourceLimit,
        "Force forecast is smaller than its retained handle");
  forecast.owned_host_bytes =
      sizeof(SelfContactTransaction) + sizeof(Impl);
  if (!Add(force.forecast.owned_host_bytes -
               sizeof(SelfContactForceAssembly),
           &forecast.owned_host_bytes) ||
      !Add(layout.bytes, &forecast.owned_host_bytes) ||
      !Add(participation.publication_host_bytes,
           &forecast.owned_host_bytes) ||
      forecast.owned_host_bytes > limits.max_host_bytes)
    return Failure(S::ResourceLimit,
        "Transaction complete host payload exceeds its cap");

  forecast.startup_host_bytes =
      sizeof(SelfContactTransaction) + sizeof(Impl);
  if (!Add(force.forecast.startup_host_bytes -
               sizeof(SelfContactForceAssembly),
           &forecast.startup_host_bytes) ||
      !Add(layout.bytes, &forecast.startup_host_bytes) ||
      !Add(participation.publication_host_bytes,
           &forecast.startup_host_bytes) ||
      forecast.startup_host_bytes > limits.max_startup_host_bytes)
    return Failure(S::ResourceLimit,
        "Transaction complete startup payload exceeds its cap");

  return {{}, forecast};
}

}  // namespace tlfea::contact
