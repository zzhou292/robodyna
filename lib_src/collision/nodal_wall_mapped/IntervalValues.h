// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "IntervalProof.h"
#include "../NodalWallContactDiagnostics.h"

#if defined(__CUDACC__)
#define TL_WALL_INTERVAL_HD __host__ __device__
#else
#define TL_WALL_INTERVAL_HD
#endif
namespace tlfea::contact::nodal_wall_mapped {
namespace interval_detail {
namespace d = nodal_wall_device_detail;
TL_WALL_INTERVAL_HD inline bool AddInterval(IntervalEndpoints& sum,
    Q4IntegralInterval term, IntervalSummary& summary) noexcept {
  Q4IntegralInterval next;
  if (!IntervalMagnitude(term.lower, summary.maximum_term) ||
      !IntervalMagnitude(term.upper, summary.maximum_term) ||
      !q4_bounds::Add(sum.Get(), term, &next)) return false;
  sum.Set(next);
  return true;
}
TL_WALL_INTERVAL_HD inline bool AddUpperTerm(double& sum, double term,
    IntervalSummary& summary) noexcept {
  return IntervalMagnitude(term, summary.maximum_term) && d::AddUpper(sum, term, &sum);
}
TL_WALL_INTERVAL_HD inline bool ObserveNode(const d::Storage& storage,
    const tl::fea::NodalPreparedView& view, unsigned index, IntervalSummary& summary) noexcept {
  const auto& node = storage.base.nodes[index];
  const auto n = node.node;
  const double force = node.force_world.x;
  const double a = view.base_kinematics.position_xyz[3*n], x = view.kinematics.position_xyz[3*n];
  const double va = view.base_kinematics.velocity_xyz[3*n], vx = view.kinematics.velocity_xyz[3*n];
  const double dx = x-a, mean = .5*(va+vx);
  Q4IntegralInterval displacement, velocity, term;
  if (!q4_bounds::Difference(x, a, &displacement) ||
      !q4_bounds::Add({va, va}, {vx, vx}, &velocity) || !q4_bounds::Scale(velocity, .5, &velocity) ||
      !d::SignedScale(velocity, force, &term) || !q4_bounds::Scale(term, view.kick_dt, &term) ||
      !AddInterval(summary.kick, term, summary) || !d::SignedScale(displacement, force, &term) ||
      !AddInterval(summary.drift, term, summary) ||
      !d::SignedScale({node.force.lower, node.force.upper}, node.wall_point.z, &term)) return false;
  if (term.lower != 0 || term.upper != 0) summary.nonzero_moments |= 1;
  if (!AddInterval(summary.moment_y, term, summary) ||
      !d::SignedScale({node.force.lower, node.force.upper}, -node.wall_point.y, &term)) return false;
  if (term.lower != 0 || term.upper != 0) summary.nonzero_moments |= 2;
  if (!AddInterval(summary.moment_z, term, summary)) return false;
  const double kick = view.kick_dt*force*mean, drift = force*dx;
  summary.kick_work += kick;
  summary.drift_work += drift;
  if (!IntervalMagnitude(kick, summary.maximum_term) ||
      !IntervalMagnitude(drift, summary.maximum_term) ||
      !IsFinite(summary.kick_work) || !IsFinite(summary.drift_work)) return false;
  const double abs_dx = ::fmax(::fabs(displacement.lower), ::fabs(displacement.upper));
  const double abs_v = ::fmax(::fabs(velocity.lower), ::fabs(velocity.upper));
  double error = 0, square = 0;
  return mass_detail::UpperProduct(storage.addition_error[index], abs_v, &error) &&
      mass_detail::UpperProduct(error, view.kick_dt, &error) &&
      AddUpperTerm(summary.addition_kick, error, summary) &&
      mass_detail::UpperProduct(storage.addition_error[index], abs_dx, &error) &&
      AddUpperTerm(summary.addition_drift, error, summary) &&
      mass_detail::UpperProduct(node.force.error, abs_dx, &error) &&
      AddUpperTerm(summary.force_uncertainty, error, summary) &&
      mass_detail::UpperProduct(abs_dx, abs_dx, &square) &&
      mass_detail::UpperProduct(node.stiffness.upper, square, &error) &&
      mass_detail::UpperProduct(.5, error, &error) &&
      AddUpperTerm(summary.quadratic, error, summary);
}
} // namespace interval_detail
TL_WALL_INTERVAL_HD inline void ObserveIntervalNode(const nodal_wall_device_detail::Storage& storage,
    const tl::fea::NodalPreparedView& view, unsigned index, IntervalSummary& summary) noexcept {
  if (!interval_detail::ObserveNode(storage, view, index, summary)) summary.serial = true;
}
TL_WALL_INTERVAL_HD inline void MergeIntervals(IntervalSummary& a, const IntervalSummary& b) noexcept {
  a.serial = a.serial || b.serial;
  a.nonzero_moments |= b.nonzero_moments;
  a.maximum_term = ::fmax(a.maximum_term, b.maximum_term);
  Q4IntegralInterval next;
  IntervalEndpoints* const targets[]{&a.kick, &a.drift, &a.moment_y, &a.moment_z};
  const IntervalEndpoints sources[]{b.kick, b.drift, b.moment_y, b.moment_z};
  for (unsigned channel = 0; channel < 4; ++channel) {
    if (!q4_bounds::Add(targets[channel]->Get(), sources[channel].Get(), &next)) a.serial = true;
    else targets[channel]->Set(next);
  }
  a.kick_work += b.kick_work;
  a.drift_work += b.drift_work;
  if (!IsFinite(a.kick_work) || !IsFinite(a.drift_work)) a.serial = true;
  double* const upper[]{&a.addition_kick, &a.addition_drift, &a.force_uncertainty, &a.quadratic};
  const double terms[]{b.addition_kick, b.addition_drift, b.force_uncertainty, b.quadratic};
  for (unsigned channel = 0; channel < 4; ++channel) {
    if (!nodal_wall_device_detail::AddUpper(*upper[channel], terms[channel], upper[channel])) a.serial = true;
  }
}
} // namespace tlfea::contact::nodal_wall_mapped
#undef TL_WALL_INTERVAL_HD
