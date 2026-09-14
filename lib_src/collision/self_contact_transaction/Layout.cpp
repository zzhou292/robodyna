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

bool Product(std::size_t a, std::size_t b,
             std::size_t* output) noexcept {
  if (!output || (a && b > SIZE_MAX / a)) return false;
  *output = a * b;
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
      config.nonlocal_policy !=
          SelfContactTransactionNonlocalPolicy::
              AcceptedSymmetricVfEeRejectIntersectionV2 ||
      config.force.configuration_id != identity.configuration_id ||
      config.force.qualification_id != identity.qualification_id ||
      !fe::shell_startup_detail::SameStartup(
          config.force.startup, identity.startup) ||
      !limits.max_candidate_triangles || !limits.max_candidate_pairs ||
      !limits.max_facet_pair_chunk ||
      !limits.max_global_events ||
      limits.max_global_events > UINT32_MAX ||
      limits.max_event_hash_slots < limits.max_global_events ||
      limits.max_candidate_pairs > SIZE_MAX / 15 ||
      !limits.max_stream_crossing_work ||
      !limits.max_host_bytes || !limits.max_device_bytes ||
      !limits.max_startup_host_bytes)
    return Failure(S::InvalidInput,
        "Transaction source, identities, startup or limits are invalid");

  const auto* surface = active_use.facets()->surface();
  const auto nodes = surface->physical()->domain()->node_count();
  const auto surface_parents = surface->parents().size();
  const auto parents = active_use.parents().size();
  const auto facets = active_use.facet_uses().size();
  const auto rigid_groups = active_use.rigid()
      ? active_use.rigid()->groups().size() : 0;
  if (!surface_parents || !parents || !facets ||
      facets > limits.max_candidate_triangles ||
      limits.max_facet_pair_chunk >
          limits.accepted_discovery.max_input_pairs ||
      limits.max_facet_pair_chunk >
          limits.candidate_discovery.max_input_pairs ||
      limits.max_facet_pair_chunk > limits.crossing.max_input_pairs ||
      limits.max_facet_pair_chunk > limits.crossing.max_results ||
      limits.max_facet_pair_chunk > SIZE_MAX / 2 ||
      2 * limits.max_facet_pair_chunk > limits.crossing.max_paths ||
      limits.crossing.max_total_work >
          limits.max_stream_crossing_work)
    return Failure(S::ResourceLimit,
        "Complete parent/facet pipeline exceeds a fixed count cap");

  const auto force = SelfContactForceAssembly::Forecast(
      config.force, active_use, limits.force);
  if (force.report.status != SelfContactForceStatus::Ok) {
    auto result = Failure(S::ForceFailure, force.report.message);
    result.report.force_status = force.report.status;
    return result;
  }
  const auto activity = SelfContactPhysicalActivity::Forecast(
      active_use, *surface->physical(), limits.activity);
  if (activity.report.status !=
      SelfContactPhysicalActivityStatus::Ok) {
    auto result = Failure(
        activity.report.status ==
                SelfContactPhysicalActivityStatus::ResourceLimit
            ? S::ResourceLimit : S::ActivityFailure,
        activity.report.message);
    result.report.activity_status = activity.report.status;
    result.report.candidate = activity.report.parent;
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
          nodes, surface_parents, parents, facets, rigid_groups,
          broadphase.forecast.pair_capacity,
          limits.max_facet_pair_chunk, config.force.event_capacity,
          limits.max_global_events, limits.max_event_hash_slots,
          limits.max_policy_outcomes,
          limits.max_host_bytes, layout))
    return Failure(S::ResourceLimit,
        "Transaction candidate arena exceeds its fixed profile");

  SelfContactTransactionForecast forecast;
  forecast.activity = activity.forecast;
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
  if (nodes > SIZE_MAX / 6)
    return Failure(S::ResourceLimit,
        "Transaction snapshot value count overflowed");
  forecast.accepted_snapshot_values = 6 * nodes;
  forecast.prepared_snapshot_values = 6 * nodes;
  forecast.broadphase_pair_capacity =
      broadphase.forecast.pair_capacity;
  forecast.candidate_triangle_capacity =
      facets;
  forecast.complete_facet_pair_capacity =
      limits.max_candidate_pairs;
  forecast.candidate_pair_capacity =
      limits.max_facet_pair_chunk;
  forecast.facet_pair_chunk_capacity =
      limits.max_facet_pair_chunk;
  forecast.feature_task_mask_capacity =
      limits.max_facet_pair_chunk;
  forecast.parent_pair_cursor_capacity =
      broadphase.forecast.pair_capacity;
  forecast.accepted_event_ledger_capacity =
      limits.max_global_events;
  forecast.event_hash_capacity =
      limits.max_event_hash_slots;
  forecast.rigid_group_snapshot_capacity = rigid_groups;
  forecast.node_rigid_group_capacity = nodes;
  forecast.parent_motion_capacity = parents;
  forecast.facet_motion_capacity = facets;
  forecast.swept_parent_bound_capacity = surface_parents;
  forecast.swept_facet_bound_capacity = facets;
  forecast.candidate_crossing_capacity =
      limits.max_facet_pair_chunk;
  forecast.accepted_event_capacity =
      config.force.event_capacity;
  forecast.accepted_certificate_capacity =
      limits.max_global_events;
  forecast.policy_outcome_capacity =
      limits.max_policy_outcomes;
  forecast.policy_chunk_capacity =
      limits.max_facet_pair_chunk;
  forecast.complete_crossing_work_capacity =
      limits.max_stream_crossing_work;
  if (!Product(forecast.broadphase_pair_capacity,
               sizeof(SelfContactPairKey),
               &forecast.broadphase_pair_readback_bytes) ||
      !Product(forecast.parent_pair_cursor_capacity,
               sizeof(sct::FacetPairCursor),
               &forecast.streaming_cursor_bytes) ||
      !Product(forecast.parent_pair_cursor_capacity,
               sizeof(std::uint32_t),
               &forecast.streaming_heap_bytes) ||
      !Product(forecast.feature_task_mask_capacity,
               sizeof(FixedTriangleFeatureTaskMask),
               &forecast.feature_task_mask_bytes))
    return Failure(S::ResourceLimit,
        "Transaction streaming byte forecast overflowed");
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
      activity.forecast.owned_host_bytes <
          sizeof(SelfContactPhysicalActivity) ||
      broadphase.forecast.owned_host_bytes <
          sizeof(SelfContactBroadphase) ||
      broadphase.forecast.retained_source_bytes >
          broadphase.forecast.owned_host_bytes -
              sizeof(SelfContactBroadphase) ||
      regularity.forecast.owned_payload_bytes <
          sizeof(SelfContactCurrentRegularity) ||
      accepted_discovery.forecast.startup_host_bytes <
          accepted_discovery.forecast.owned_host_bytes ||
      candidate_discovery.forecast.startup_host_bytes <
          candidate_discovery.forecast.owned_host_bytes ||
      activity.forecast.startup_host_bytes <
          activity.forecast.owned_host_bytes ||
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
  if (!Add(activity.forecast.owned_host_bytes -
               sizeof(SelfContactPhysicalActivity),
           &forecast.owned_host_bytes) ||
      !Add(force.forecast.owned_host_bytes -
               sizeof(SelfContactForceAssembly),
           &forecast.owned_host_bytes) ||
      !Add(layout.bytes, &forecast.owned_host_bytes) ||
      !Add(broadphase.forecast.owned_host_bytes -
               sizeof(SelfContactBroadphase) -
               broadphase.forecast.retained_source_bytes,
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

  const std::size_t force_startup_delta =
      force.forecast.startup_host_bytes -
      force.forecast.owned_host_bytes;
  if (force.forecast.retained_active_use_bytes >
          force_startup_delta ||
      !Add(broadphase.forecast.retained_source_bytes,
           &forecast.shared_backing_discount_bytes) ||
      !Add(force.forecast.retained_active_use_bytes,
           &forecast.shared_backing_discount_bytes))
    return Failure(S::ResourceLimit,
        "Shared transaction backing discount is invalid");
  const std::size_t startup_scratch = std::max({
      force_startup_delta -
          force.forecast.retained_active_use_bytes,
      activity.forecast.startup_host_bytes -
          activity.forecast.owned_host_bytes,
      broadphase.forecast.startup_host_bytes -
          broadphase.forecast.owned_host_bytes,
      regularity.forecast.startup_payload_bytes -
          regularity.forecast.owned_payload_bytes});
  forecast.startup_host_bytes = forecast.owned_host_bytes;
  if (!Add(accepted_discovery.forecast.startup_host_bytes -
               accepted_discovery.forecast.owned_host_bytes,
           &forecast.startup_host_bytes) ||
      !Add(candidate_discovery.forecast.startup_host_bytes -
               candidate_discovery.forecast.owned_host_bytes,
           &forecast.startup_host_bytes) ||
      !Add(startup_scratch, &forecast.startup_host_bytes) ||
      forecast.startup_host_bytes > limits.max_startup_host_bytes)
    return Failure(S::ResourceLimit,
        "Transaction complete startup payload exceeds its cap");

  return {{}, forecast};
}

}  // namespace tlfea::contact
