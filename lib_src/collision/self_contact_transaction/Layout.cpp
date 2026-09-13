// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"

#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>
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
      config.broadphase_axis > 2 ||
      config.activity_policy !=
          SelfContactTransactionActivityPolicy::
              RequireAllSelectedParentsActiveV1 ||
      config.nonlocal_policy !=
          SelfContactTransactionNonlocalPolicy::
              AcceptedVertexFaceOnlyRejectIntersectionAndEdgeV1 ||
      config.force.configuration_id != identity.configuration_id ||
      config.force.qualification_id != identity.qualification_id ||
      !fe::shell_startup_detail::SameStartup(
          config.force.startup, identity.startup) ||
      !limits.max_candidate_triangles || !limits.max_candidate_pairs ||
      !limits.max_host_bytes || !limits.max_device_bytes ||
      !limits.max_startup_host_bytes)
    return Failure(S::InvalidInput,
        "Transaction source, identities, startup or limits are invalid");

  const auto* surface = active_use.facets()->surface();
  const auto nodes = surface->physical()->domain()->node_count();
  const auto surface_parents = surface->parents().size();
  const auto parents = active_use.parents().size();
  const auto facets = active_use.facet_uses().size();
  if (!surface_parents || !parents || !facets ||
      facets > limits.max_candidate_triangles ||
      facets > limits.crossing.max_paths ||
      limits.max_candidate_pairs >
          limits.accepted_discovery.max_input_pairs ||
      limits.max_candidate_pairs >
          limits.candidate_discovery.max_input_pairs ||
      limits.max_candidate_pairs > limits.crossing.max_input_pairs ||
      limits.max_candidate_pairs > limits.crossing.max_results)
    return Failure(S::ResourceLimit,
        "Complete parent/facet pipeline exceeds a fixed count cap");

  const auto force = SelfContactForceAssembly::Forecast(
      config.force, active_use, limits.force);
  if (force.report.status != SelfContactForceStatus::Ok) {
    auto result = Failure(S::ForceFailure, force.report.message);
    result.report.force_status = force.report.status;
    return result;
  }

  const auto broadphase =
      SelfContactBroadphase::Preflight(*surface, limits.broadphase);
  if (broadphase.report.status != SelfContactBroadphaseStatus::Ok) {
    auto result = Failure(
        broadphase.report.status ==
                SelfContactBroadphaseStatus::ResourceLimit
            ? S::ResourceLimit : S::BroadphaseFailure,
        broadphase.report.message);
    result.report.broadphase_status = broadphase.report.status;
    return result;
  }
  const auto accepted_discovery =
      FixedTriangleFeatureDiscovery::Preflight(
          limits.accepted_discovery);
  if (accepted_discovery.report.status !=
      FixedTriangleDiscoveryStatus::Ok) {
    auto result = Failure(S::DiscoveryFailure,
                          accepted_discovery.report.message);
    result.report.discovery_status =
        accepted_discovery.report.status;
    return result;
  }
  const auto candidate_discovery =
      FixedTriangleFeatureDiscovery::Preflight(
          limits.candidate_discovery);
  if (candidate_discovery.report.status !=
      FixedTriangleDiscoveryStatus::Ok) {
    auto result = Failure(S::DiscoveryFailure,
                          candidate_discovery.report.message);
    result.report.discovery_status =
        candidate_discovery.report.status;
    return result;
  }
  const auto regularity = SelfContactCurrentRegularity::Preflight(
      active_use, limits.regularity);
  if (regularity.report.status !=
      SelfContactCurrentRegularityStatus::Ok) {
    auto result = Failure(S::RegularityFailure,
                          regularity.report.message);
    result.report.regularity_status = regularity.report.status;
    return result;
  }
  const auto crossing =
      RepresentedIntervalCrossing::Preflight(limits.crossing);
  if (crossing.report.status != RepresentedIntervalStatus::Ok) {
    auto result = Failure(S::CrossingFailure,
                          crossing.report.message);
    result.report.crossing_status = crossing.report.status;
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

  sct::Layout layout;
  if (!sct::MakeLayout(
          nodes, surface_parents, parents, facets,
          broadphase.forecast.pair_capacity,
          limits.max_candidate_pairs, config.force.event_capacity,
          limits.max_host_bytes, layout))
    return Failure(S::ResourceLimit,
        "Transaction candidate arena exceeds its fixed profile");

  SelfContactTransactionForecast forecast;
  forecast.force = force.forecast;
  forecast.broadphase = broadphase.forecast;
  forecast.accepted_discovery = accepted_discovery.forecast;
  forecast.candidate_discovery = candidate_discovery.forecast;
  forecast.regularity = regularity.forecast;
  forecast.crossing = crossing.forecast;
  forecast.participation = participation;
  forecast.surface_parent_map_capacity = surface_parents;
  forecast.parent_facet_offset_count = parents + 1;
  forecast.facet_descriptor_capacity = facets;
  forecast.parent_activity_bytes = 2 * parents;
  if (nodes > SIZE_MAX / 6)
    return Failure(S::ResourceLimit,
        "Transaction snapshot value count overflowed");
  forecast.accepted_snapshot_values = 6 * nodes;
  forecast.prepared_snapshot_values = 6 * nodes;
  forecast.broadphase_pair_capacity =
      broadphase.forecast.pair_capacity;
  forecast.candidate_triangle_capacity =
      facets;
  forecast.candidate_pair_capacity = limits.max_candidate_pairs;
  forecast.accepted_event_capacity =
      config.force.event_capacity;
  forecast.accepted_certificate_capacity =
      config.force.event_capacity;
  forecast.policy_outcome_capacity = limits.max_candidate_pairs;
  forecast.candidate_arena_bytes = layout.bytes;
  if (!Add(force.forecast.device_bytes, &forecast.device_bytes) ||
      !Add(broadphase.forecast.device_bytes,
           &forecast.device_bytes) ||
      !Add(force.forecast.device_allocations,
           &forecast.device_allocations) ||
      !Add(1, &forecast.device_allocations) ||
      forecast.device_bytes > limits.max_device_bytes)
    return Failure(S::ResourceLimit,
        "Transaction device payload exceeds its complete cap");

  if (force.forecast.owned_host_bytes <
      sizeof(SelfContactForceAssembly) ||
      broadphase.forecast.owned_host_bytes <
          sizeof(SelfContactBroadphase) ||
      regularity.forecast.owned_payload_bytes <
          sizeof(SelfContactCurrentRegularity) ||
      force.forecast.startup_host_bytes <
          force.forecast.owned_host_bytes ||
      broadphase.forecast.startup_host_bytes <
          broadphase.forecast.owned_host_bytes ||
      regularity.forecast.startup_payload_bytes <
          regularity.forecast.owned_payload_bytes)
    return Failure(S::ResourceLimit,
        "A component forecast is smaller than its retained handle");
  forecast.owned_host_bytes =
      sizeof(SelfContactTransaction) + sizeof(Impl);
  if (!Add(force.forecast.owned_host_bytes -
               sizeof(SelfContactForceAssembly),
           &forecast.owned_host_bytes) ||
      !Add(layout.bytes, &forecast.owned_host_bytes) ||
      !Add(broadphase.forecast.owned_host_bytes -
               sizeof(SelfContactBroadphase),
           &forecast.owned_host_bytes) ||
      !Add(accepted_discovery.forecast.owned_host_bytes,
           &forecast.owned_host_bytes) ||
      !Add(candidate_discovery.forecast.owned_host_bytes,
           &forecast.owned_host_bytes) ||
      !Add(regularity.forecast.owned_payload_bytes -
               sizeof(SelfContactCurrentRegularity),
           &forecast.owned_host_bytes) ||
      !Add(crossing.forecast.owned_host_bytes,
           &forecast.owned_host_bytes) ||
      !Add(participation.publication_host_bytes,
           &forecast.owned_host_bytes) ||
      forecast.owned_host_bytes > limits.max_host_bytes)
    return Failure(S::ResourceLimit,
        "Transaction complete host payload exceeds its cap");

  const std::size_t startup_scratch = std::max({
      force.forecast.startup_host_bytes -
          force.forecast.owned_host_bytes,
      broadphase.forecast.startup_host_bytes -
          broadphase.forecast.owned_host_bytes,
      regularity.forecast.startup_payload_bytes -
          regularity.forecast.owned_payload_bytes});
  forecast.startup_host_bytes = forecast.owned_host_bytes;
  if (!Add(startup_scratch, &forecast.startup_host_bytes) ||
      forecast.startup_host_bytes > limits.max_startup_host_bytes)
    return Failure(S::ResourceLimit,
        "Transaction complete startup payload exceeds its cap");

  return {{}, forecast};
}

}  // namespace tlfea::contact
