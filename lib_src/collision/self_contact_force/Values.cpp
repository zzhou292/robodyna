// SPDX-License-Identifier: AGPL-3.0-or-later
#include "../SelfContactForceValues.h"
#include "../fixed_triangle_features/Geometry.h"
#include "lib_src/solvers/NodalTrialIdentity.h"

#include <algorithm>

namespace tlfea::contact {
namespace {

bool EventLess(const SelfContactForceEvent& a,
               const SelfContactForceEvent& b) noexcept {
  const int feature =
      fixed_triangle_features::Compare(a.feature, b.feature);
  return feature < 0 || (!feature && a.source_order < b.source_order);
}

bool IncidenceLess(const SelfContactForceIncidence& a,
                   const SelfContactForceIncidence& b) noexcept {
  return a.node < b.node || (a.node == b.node && a.event < b.event);
}

}  // namespace

SelfContactForceReport BuildSelfContactForceIncidence(
    SelfContactForceEvent* events, std::size_t event_count,
    std::uint32_t node_count,
    SelfContactForceIncidence* incidences, std::size_t incidence_capacity,
    SelfContactForceNodeIncidence* nodes, std::size_t node_capacity,
    SelfContactForceIncidenceSummary* summary) noexcept {
  using S = SelfContactForceStatus;
  if (!events || !event_count || !node_count || !incidences || !nodes ||
      !summary || event_count > UINT32_MAX ||
      event_count > SIZE_MAX / sizeof(*events) ||
      incidence_capacity > SIZE_MAX / sizeof(*incidences) ||
      node_capacity > SIZE_MAX / sizeof(*nodes))
    return {S::InvalidInput, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput,
            tl::fea::NodalStatus::Ok,
            "Incidence inputs are invalid"};
  using tl::fea::trial_identity::Disjoint;
  const auto event_bytes = event_count * sizeof(*events);
  const auto incidence_bytes = incidence_capacity * sizeof(*incidences);
  const auto node_bytes = node_capacity * sizeof(*nodes);
  if (!Disjoint(summary, sizeof(*summary), events, event_bytes) ||
      !Disjoint(summary, sizeof(*summary), incidences, incidence_bytes) ||
      !Disjoint(summary, sizeof(*summary), nodes, node_bytes) ||
      !Disjoint(events, event_bytes, incidences, incidence_bytes) ||
      !Disjoint(events, event_bytes, nodes, node_bytes) ||
      !Disjoint(incidences, incidence_bytes, nodes, node_bytes))
    return {S::InvalidInput, SIZE_MAX, UINT64_MAX, UINT32_MAX,
            SurfacePenaltyStatus::InvalidInput,
            tl::fea::NodalStatus::Ok,
            "Incidence scratch/output ranges overlap"};

  std::sort(events, events + event_count, EventLess);
  for (std::size_t event = 0; event < event_count; ++event) {
    if (events[event].source_order == UINT64_MAX)
      return {S::InvalidInput, event, events[event].source_order, UINT32_MAX,
              SurfacePenaltyStatus::InvalidInput,
              tl::fea::NodalStatus::Ok,
              "Event source order is invalid"};
    if (event &&
        fixed_triangle_features::Compare(events[event - 1].feature,
                                         events[event].feature) == 0)
      return {S::DuplicateEvent, event, events[event].source_order,
              UINT32_MAX, SurfacePenaltyStatus::InvalidInput,
              tl::fea::NodalStatus::Ok,
              "Duplicate canonical self-contact feature"};
  }

  std::size_t incidence_count = 0;
  for (std::size_t event = 0; event < event_count; ++event) {
    std::uint32_t unique[8]{};
    std::uint32_t unique_count = 0;
    for (unsigned endpoint = 0; endpoint < 2; ++endpoint) {
      const auto& point = events[event].endpoints[endpoint];
      const auto checked = ValidateWeightedSurfacePoint(point, node_count);
      if (checked != Status::kOk)
        return {S::InvalidInput, event, events[event].source_order,
                UINT32_MAX, SurfacePenaltyStatus::InvalidInput,
                tl::fea::NodalStatus::Ok,
                "Event endpoint map is invalid"};
      for (std::uint32_t slot = 0; slot < point.count; ++slot) {
        const auto node = point.nodes[slot];
        std::uint32_t i = 0;
        while (i < unique_count && unique[i] != node) ++i;
        if (i == unique_count) unique[unique_count++] = node;
      }
    }
    if (unique_count > incidence_capacity - incidence_count)
      return {S::ResourceLimit, event, events[event].source_order,
              UINT32_MAX, SurfacePenaltyStatus::OutOfRange,
              tl::fea::NodalStatus::Ok,
              "Event-node incidence capacity is exhausted"};
    for (std::uint32_t i = 0; i < unique_count; ++i)
      incidences[incidence_count++] = {
          unique[i], static_cast<std::uint32_t>(event)};
  }

  std::sort(incidences, incidences + incidence_count, IncidenceLess);
  std::size_t node_count_used = 0;
  for (std::size_t offset = 0; offset < incidence_count;) {
    if (node_count_used == node_capacity)
      return {S::ResourceLimit, SIZE_MAX, UINT64_MAX,
              incidences[offset].node, SurfacePenaltyStatus::OutOfRange,
              tl::fea::NodalStatus::Ok,
              "Touched-node capacity is exhausted"};
    const auto node = incidences[offset].node;
    std::size_t end = offset + 1;
    while (end < incidence_count && incidences[end].node == node) ++end;
    if (offset > UINT32_MAX || end - offset > UINT32_MAX)
      return {S::ResourceLimit, SIZE_MAX, UINT64_MAX, node,
              SurfacePenaltyStatus::OutOfRange,
              tl::fea::NodalStatus::Ok,
              "Node-incidence range is unrepresentable"};
    nodes[node_count_used++] = {
        node, static_cast<std::uint32_t>(offset),
        static_cast<std::uint32_t>(end - offset)};
    offset = end;
  }
  *summary = {incidence_count, node_count_used};
  return {};
}

}  // namespace tlfea::contact
