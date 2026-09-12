// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"

namespace tlfea::contact {
// No clamping, sorting or renormalization. Pointee lengths, lifetime and
// nonoverlap are caller contracts; all pointers occupy one execution space.
// Every failure preserves the complete output. Arithmetic is binary64 RN,
// without reassociation, FMA contraction, fast-math or flush-to-zero.
TL_SURFACE_HD inline Status ValidateWeightedSurfacePoint(
    const WeightedSurfacePoint& point, std::uint32_t node_count) {
  if (!node_count || (point.count != 3 && point.count != 4))
    return Status::kInvalidArgument;
  double sum = 0;
  for (std::uint32_t i = 0; i < point.count; ++i) {
    const double weight = point.weights[i];
    if (!IsFinite(weight) || weight < 0 || weight > 1)
      return Status::kInvalidArgument;
    if (point.nodes[i] >= node_count) return Status::kOutOfRange;
    for (std::uint32_t j = 0; j < i; ++j)
      if (point.nodes[i] == point.nodes[j]) return Status::kInvalidArgument;
    sum += weight;
  }
  return ::fabs(sum - 1) <= 1e-12 ? Status::kOk : Status::kInvalidArgument;
}

// Also suitable for a caller that has only current positions. Every consumed
// node is checked, including zero weights. The source-slot Add/Scale order is
// the same as the existing Q4 and linear-triangle interpolation arithmetic.
TL_SURFACE_HD inline Status EvaluateWeightedSurfacePosition(
    VectorView values, const WeightedSurfacePoint& point, Vec3* output) {
  if (!output || !values.valid()) return Status::kInvalidArgument;
  const auto status = ValidateWeightedSurfacePoint(point, values.node_count);
  if (status != Status::kOk) return status;
  Vec3 result;
  for (std::uint32_t i = 0; i < point.count; ++i) {
    const Vec3 value = values.at(point.nodes[i]);
    if (!IsFinite(value)) return Status::kInvalidArgument;
    result = Add(result, Scale(value, point.weights[i]));
  }
  if (!IsFinite(result)) return Status::kNonFiniteResult;
  *output = result;
  return Status::kOk;
}

TL_SURFACE_HD inline Status EvaluateWeightedSurfacePoint(
    VectorView positions, VectorView velocities, const WeightedSurfacePoint& point,
    WeightedPointKinematics* output) {
  if (!output || !positions.valid() || !velocities.valid() ||
      positions.node_count != velocities.node_count)
    return Status::kInvalidArgument;
  WeightedPointKinematics result;
  auto status = EvaluateWeightedSurfacePosition(positions, point, &result.position);
  if (status != Status::kOk) return status;
  status = EvaluateWeightedSurfacePosition(velocities, point, &result.velocity);
  if (status != Status::kOk) return status;
  *output = result;
  return Status::kOk;
}

TL_SURFACE_HD inline Status ProjectWeightedSurfaceForce(
    const WeightedSurfacePoint& point, std::uint32_t node_count, Vec3 force,
    WeightedNodalForces* output) {
  if (!output || !IsFinite(force)) return Status::kInvalidArgument;
  const auto status = ValidateWeightedSurfacePoint(point, node_count);
  if (status != Status::kOk) return status;
  WeightedNodalForces result;
  result.count = point.count;
  for (std::uint32_t i = 0; i < point.count; ++i) {
    result.nodes[i] = point.nodes[i];
    result.forces[i] = Scale(force, point.weights[i]);
    if (!IsFinite(result.forces[i])) return Status::kNonFiniteResult;
  }
  *output = result;
  return Status::kOk;
}
} // namespace tlfea::contact
