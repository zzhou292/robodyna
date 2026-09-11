// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ObserverTypes.h"
#include <cfloat>

#if defined(__CUDACC__)
#define TL_MAPPED_OBSERVER_HD __host__ __device__
#else
#define TL_MAPPED_OBSERVER_HD
#endif

namespace tl::fea::mapped_shell {
TL_MAPPED_OBSERVER_HD inline void AddObservation(ObserverSummary& out, unsigned channel, double term) noexcept {
  if (!tl::math::Finite(term)) {
    out.serial = true;
    return;
  }
  out.maximum_term = ::fmax(out.maximum_term, ::fabs(term));
  out.sum[channel] += term;
  if (!tl::math::Finite(out.sum[channel])) out.serial = true;
}
// Retain each already-rounded three/four-slot cache-work term. The shared leaf
// keeps its original serial -= expression for its default double accumulator.
struct WorkObservation {
  ObserverSummary& out;
  unsigned channel;
  TL_MAPPED_OBSERVER_HD void operator-=(double term) noexcept {
    AddObservation(out, channel, -term);
  }
};
TL_MAPPED_OBSERVER_HD inline void MergeObservations(ObserverSummary& a, const ObserverSummary& b) noexcept {
  a.serial = a.serial || b.serial;
  a.maximum_term = ::fmax(a.maximum_term, b.maximum_term);
  for (unsigned channel = 0; channel < ObserverChannels; ++channel) {
    a.sum[channel] += b.sum[channel];
    if (!tl::math::Finite(a.sum[channel])) a.serial = true;
  }
  if (b.material_count) {
    a.minimum_area = a.material_count ? ::fmin(a.minimum_area, b.minimum_area) : b.minimum_area;
    a.minimum_thickness = a.material_count ? ::fmin(a.minimum_thickness, b.minimum_thickness) : b.minimum_thickness;
    a.minimum_dt = a.material_count ? ::fmin(a.minimum_dt, b.minimum_dt) : b.minimum_dt;
  }
  a.material_count += b.material_count;
  a.maximum_displacement = ::fmax(a.maximum_displacement, b.maximum_displacement);
  a.maximum_strain = ::fmax(a.maximum_strain, b.maximum_strain);
  a.maximum_curvature = ::fmax(a.maximum_curvature, b.maximum_curvature);
}
// Existing QEPH sufficient bound also covers T3's three slot terms per parent.
// All original serial prefixes and tree partials stay below DBL_MAX/8; the
// factor-eight margin covers binary64 addition and rounded-division error.
// An inadequate proof requests the original serial path, not rejection.
TL_MAPPED_OBSERVER_HD inline bool FiniteObserverPrefixes(std::size_t parents, double maximum_term) noexcept {
  if (!parents || parents > UINT32_MAX / 4 || !tl::math::Finite(maximum_term) || maximum_term < 0) return false;
  const double terms = 4.0 * static_cast<double>(parents) + 1;
  return maximum_term <= DBL_MAX / (8.0 * terms);
}
} // namespace tl::fea::mapped_shell

#undef TL_MAPPED_OBSERVER_HD
