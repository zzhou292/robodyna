// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Type45History.h"
#include "Type45Kinematics.h"
#include "Type45Reduction.h"

namespace tl::fea::type45 {
// Supplied current endpoint positions, interval spin and caller stamp. No new
// timestep selection or dynamics clock; the caller chooses whether to accept
// the staged history. A failed call preserves every output, including aliasing.
TL_TYPE45_HD inline Status Evaluate(const Reference& reference, const History& accepted,
                                   const Interval& interval, Evaluation& output) {
  using namespace detail;
  if (!accepted.Matches(reference)) return Status::ReferenceMismatch;
  if (!Valid(accepted.values_)) return Status::InvalidHistory;
  if (!Positive(interval.dt_s) || !Same(interval.base_time_s,accepted.stamp_.time_s) ||
      accepted.stamp_.sample_index==UINT64_MAX ||
      interval.sample_index!=accepted.stamp_.sample_index+1) return Status::StaleInterval;
  const double end=interval.base_time_s+interval.dt_s;
  if (!Finite(end) || end<=interval.base_time_s) return Status::StaleInterval;
  for (unsigned i=0; i<2; ++i)
    if (!Finite(interval.position_m[i]) || !Finite(interval.angular_velocity_rad_s[i]))
      return Status::InvalidGeometry;
  Evaluation next;
  next.history=accepted;
  auto& values=next.history.values_;
  if (!AdvanceKinematics(reference,accepted.values_,interval,values,next.diagnostics) ||
      !Response(reference,accepted.values_,values,next.diagnostics) ||
      !Reduce(reference,values,next.diagnostics,next.endpoint) || !Valid(values))
    return Status::NonfiniteResult;
  next.history.stamp_={end,interval.sample_index};
  output=next;
  return Status::Success;
}
} // namespace tl::fea::type45
