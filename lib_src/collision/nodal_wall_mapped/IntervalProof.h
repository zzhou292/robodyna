// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "IntervalTypes.h"
#include "../NodalWallContactStorage.h"
#include <cfloat>

#if defined(__CUDACC__)
#define TL_WALL_INTERVAL_HD __host__ __device__
#else
#define TL_WALL_INTERVAL_HD
#endif
namespace tlfea::contact::nodal_wall_mapped {
TL_WALL_INTERVAL_HD inline bool IntervalMagnitude(double value, double& maximum) noexcept {
  if (!IsFinite(value)) return false;
  maximum = ::fmax(maximum, ::fabs(value));
  return true;
}

// Bounds the difference between the old serial and new directed endpoint sums.
// Each consumes the same rounded leaf endpoints. See the owning README proof.
TL_WALL_INTERVAL_HD inline bool SerialEndpointError(std::size_t nodes, double maximum,
    double& error) noexcept {
  double absolute = 0, roundoff = 0, subnormal = 0;
  const double count = static_cast<double>(nodes);
  const double factor = 16.0*(count+32);
  return mass_detail::UpperProduct(count, maximum, &absolute) &&
      mass_detail::UpperProduct(factor*0x1p-53, absolute, &roundoff) &&
      mass_detail::UpperProduct(factor, DBL_TRUE_MIN, &subnormal) &&
      nodal_wall_device_detail::AddUpper(roundoff, subnormal, &error);
}
TL_WALL_INTERVAL_HD inline bool SerialEndpointScalesNormally(double endpoint,
    double error, double duration) noexcept {
  Q4IntegralInterval low, high, scaled;
  if (!q4_bounds::Difference(endpoint, error, &low) ||
      !q4_bounds::Add({endpoint, endpoint}, {error, error}, &high)) return false;
  const Q4IntegralInterval possible{low.lower, high.upper};
  if (possible.lower <= 0 && possible.upper >= 0) return false;
  if (!q4_bounds::Scale(possible, duration, &scaled)) return false;
  const double minimum = ::fmin(::fabs(scaled.lower), ::fabs(scaled.upper));
  const double maximum = ::fmax(::fabs(scaled.lower), ::fabs(scaled.upper));
  return minimum > DBL_MIN && maximum < DBL_MAX/8;
}
TL_WALL_INTERVAL_HD inline bool SerialMomentsRemainAdmitted(const IntervalSummary& summary,
    std::size_t nodes, double maximum, double duration) noexcept {
  if (duration == 0 || !summary.nonzero_moments) return true;
  double error = 0;
  if (!SerialEndpointError(nodes, maximum, error)) return false;
  const IntervalEndpoints endpoints[]{summary.moment_y, summary.moment_z};
  for (unsigned channel = 0; channel < 2; ++channel) {
    if (!(summary.nonzero_moments & (1u << channel))) continue;
    if (!SerialEndpointScalesNormally(endpoints[channel].lower, error, duration) ||
        !SerialEndpointScalesNormally(endpoints[channel].upper, error, duration)) return false;
  }
  return true;
}
TL_WALL_INTERVAL_HD inline bool IntervalSerialDomain(const IntervalSummary& summary,
    std::size_t nodes, const NodalWallDiagnostics& base, const NodalWallDiagnostics& current,
    double duration) noexcept {
  if (summary.serial || !ObserverBlocks(nodes) || !IsFinite(duration) || duration < 0) return false;
  // These are exactly the consumed post-loop seeds. Other diagnostic fields
  // must not gain new finite checks. Negative error seeds use the old domain.
  if (base.potential.error < 0 || current.potential.error < 0) return false;
  const double seeds[]{base.potential.value, base.potential.error,
      current.potential.value, current.potential.error, current.kick_work, current.drift_work,
      base.resultant.lower, base.resultant.upper, base.wall_reaction.x,
      base.wall_moment.y, base.wall_moment.z,
      duration*base.wall_reaction.x, duration*base.wall_moment.y, duration*base.wall_moment.z};
  double maximum = summary.maximum_term;
  for (double value : seeds) if (!IntervalMagnitude(value, maximum)) return false;
  if (!IsFinite(maximum) || maximum < 0 ||
      maximum > DBL_MAX/(64.0*(static_cast<double>(nodes)+1))) return false;
  return SerialMomentsRemainAdmitted(summary, nodes, maximum, duration);
}
} // namespace tlfea::contact::nodal_wall_mapped
#undef TL_WALL_INTERVAL_HD
