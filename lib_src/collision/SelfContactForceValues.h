// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactForceTypes.h"
#include "Q4ContactBounds.h"
#include "fixed_triangle_features/Geometry.h"

namespace tlfea::contact {

inline int CompareSelfContactForceEventIdentity(
    const SelfContactForceEvent& a,
    const SelfContactForceEvent& b) noexcept {
  const int feature =
      fixed_triangle_features::Compare(a.feature, b.feature);
  if (feature) return feature;
  for (unsigned side = 0; side < 2; ++side) {
    if (a.classification.parent[side] <
        b.classification.parent[side])
      return -1;
    if (b.classification.parent[side] <
        a.classification.parent[side])
      return 1;
  }
  return 0;
}

inline bool SameSelfContactForceEventIdentity(
    const SelfContactForceEvent& a,
    const SelfContactForceEvent& b) noexcept {
  return CompareSelfContactForceEventIdentity(a, b) == 0;
}

TL_SURFACE_HD inline bool PositiveSelfContactArea(
    Q4CertifiedIntegral area) noexcept {
  Q4CertifiedIntegral checked;
  return IsFinite(area.error) && area.error >= 0 &&
      area.value > 0 && area.lower > 0 &&
      q4_bounds::Certify(area.value, {area.lower, area.upper}, &checked) &&
      checked.error <= area.error;
}

// The represented force coefficient is the outward binary64 product of the
// explicit pressure stiffness and the active-use certified directed area.
// A lost positive product and overflow both reject.
TL_SURFACE_HD inline bool RepresentedSelfContactStiffness(
    double stiffness_per_area_n_m3, Q4CertifiedIntegral area,
    double* stiffness_n_m) noexcept {
  if (!stiffness_n_m || !IsFinite(stiffness_per_area_n_m3) ||
      stiffness_per_area_n_m3 <= 0 || !PositiveSelfContactArea(area))
    return false;
  double next = 0;
  if (!mass_detail::UpperProduct(stiffness_per_area_n_m3, area.value, &next) ||
      !IsFinite(next) || next <= 0)
    return false;
  *stiffness_n_m = next;
  return true;
}

// Scratch-only host canonicalization. An empty event batch publishes an exact
// zero summary without requiring event/incidence/node storage. Nonempty events
// sort by immutable feature plus ordered active-parent ownership, then source
// order. Incidence sorts by physical node and canonical event ordinal; one
// event contributes at most once to one physical node even when its two
// endpoint maps share that node.
SelfContactForceReport BuildSelfContactForceIncidence(
    SelfContactForceEvent* events, std::size_t event_count,
    std::uint32_t node_count,
    SelfContactForceIncidence* incidences, std::size_t incidence_capacity,
    SelfContactForceNodeIncidence* nodes, std::size_t node_capacity,
    SelfContactForceIncidenceSummary* summary) noexcept;

}  // namespace tlfea::contact
