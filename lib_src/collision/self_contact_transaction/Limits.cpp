// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../SelfContactTransactionTypes.h"

#include <algorithm>

namespace tlfea::contact {
namespace {

bool Product(std::size_t a, std::size_t b,
             std::size_t* output) noexcept {
  if (!output || (a && b > SIZE_MAX / a)) return false;
  *output = a * b;
  return true;
}

}  // namespace

SelfContactTransactionLimits SelfContactTransactionLimits::Vehicle(
    const ExactCensus& census, std::size_t facet_pair_chunk,
    std::size_t event_ledger_capacity,
    std::size_t event_hash_slots,
    std::size_t policy_outcome_capacity,
    std::size_t crossing_work_per_pair,
    std::size_t crossing_work_per_chunk,
    std::size_t crossing_work_complete,
    unsigned crossing_depth,
    std::size_t max_host_bytes,
    std::size_t max_device_bytes,
    std::size_t max_startup_host_bytes,
    unsigned discovery_worker_count,
    unsigned crossing_worker_count) noexcept {
  SelfContactTransactionLimits result;
  const auto invalid = [&]() noexcept {
    result.max_host_bytes = 0;
    result.max_device_bytes = 0;
    result.max_startup_host_bytes = 0;
    return result;
  };
  std::size_t twice_chunk = 0;
  std::size_t six_chunk = 0;
  std::size_t fifteen_chunk = 0;
  if (!census.nodes || !census.surface_parents ||
      !census.selected_parents || !census.maximum_family_parents ||
      !census.facets || !facet_pair_chunk ||
      census.facet_pairs > SIZE_MAX / 15 ||
      !event_ledger_capacity ||
      census.accepted_events > event_ledger_capacity ||
      event_ledger_capacity > UINT32_MAX ||
      event_hash_slots < event_ledger_capacity ||
      !crossing_work_per_pair || !crossing_work_per_chunk ||
      !crossing_work_complete ||
      crossing_work_per_chunk > crossing_work_complete ||
      crossing_depth > 52 || !max_host_bytes ||
      !max_device_bytes || !max_startup_host_bytes ||
      !discovery_worker_count ||
      discovery_worker_count >
          FixedTriangleFeatureMaximumWorkerCount ||
      !crossing_worker_count ||
      crossing_worker_count >
          RepresentedIntervalMaximumWorkerCount ||
      !Product(facet_pair_chunk, 2, &twice_chunk) ||
      !Product(facet_pair_chunk, 6, &six_chunk) ||
      !Product(facet_pair_chunk, 15, &fifteen_chunk))
    return invalid();

  const auto parent_pair_capacity =
      std::max<std::size_t>(1, census.parent_pairs);
  const auto facet_pair_capacity =
      std::max<std::size_t>(1, census.facet_pairs);
  result.activity = {
      census.selected_parents, census.maximum_family_parents,
      max_host_bytes, max_startup_host_bytes};
  result.force = {
      event_ledger_capacity, census.nodes,
      max_host_bytes, max_device_bytes, max_startup_host_bytes};
  result.broadphase = {
      census.surface_parents, census.nodes, parent_pair_capacity,
      max_device_bytes, max_host_bytes};
  result.accepted_discovery = {
      facet_pair_chunk, twice_chunk, six_chunk, six_chunk,
      fifteen_chunk, fifteen_chunk,
      facet_pair_chunk, facet_pair_chunk, max_host_bytes,
      discovery_worker_count};
  result.candidate_discovery = result.accepted_discovery;
  result.regularity = {
      census.selected_parents, census.facets, max_host_bytes};
  result.crossing = {
      twice_chunk, facet_pair_chunk, facet_pair_chunk,
      crossing_work_per_pair, crossing_work_per_chunk,
      crossing_depth, max_host_bytes, crossing_worker_count};
  result.max_candidate_triangles = census.facets;
  result.max_candidate_pairs = facet_pair_capacity;
  result.max_facet_pair_chunk = facet_pair_chunk;
  result.max_global_events = event_ledger_capacity;
  result.max_event_hash_slots = event_hash_slots;
  result.max_policy_outcomes = policy_outcome_capacity;
  result.max_stream_crossing_work = crossing_work_complete;
  result.max_host_bytes = max_host_bytes;
  result.max_device_bytes = max_device_bytes;
  result.max_startup_host_bytes = max_startup_host_bytes;
  return result;
}

}  // namespace tlfea::contact
