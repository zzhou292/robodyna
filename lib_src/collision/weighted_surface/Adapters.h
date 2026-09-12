// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Mapping.h"
#include "../Q4SurfaceMapping.h"

namespace tlfea::contact {
// These numeric adapters retain original node order. They do not authenticate
// a source parent or admit reference offsets; that remains the binding's job.
TL_SURFACE_HD inline Status MakeWeightedQ4Point(const std::uint32_t (&nodes)[4],
    std::uint32_t node_count, double u, double v, WeightedSurfacePoint* output) {
  if (!output) return Status::kInvalidArgument;
  WeightedSurfacePoint result;
  result.count = 4;
  for (unsigned i = 0; i < 4; ++i) result.nodes[i] = nodes[i];
  auto status = EvaluateQ4Shape(u, v, result.weights);
  if (status != Status::kOk) return status;
  status = ValidateWeightedSurfacePoint(result, node_count);
  if (status != Status::kOk) return status;
  *output = result;
  return Status::kOk;
}
TL_SURFACE_HD inline Status MakeWeightedT3Point(const std::uint32_t (&nodes)[3],
    std::uint32_t node_count, const double (&weights)[3], WeightedSurfacePoint* output) {
  if (!output) return Status::kInvalidArgument;
  WeightedSurfacePoint result;
  result.count = 3;
  for (unsigned i = 0; i < 3; ++i) {
    result.nodes[i] = nodes[i];
    result.weights[i] = weights[i];
  }
  const auto status = ValidateWeightedSurfacePoint(result, node_count);
  if (status != Status::kOk) return status;
  *output = result;
  return Status::kOk;
}
} // namespace tlfea::contact
