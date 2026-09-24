// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Storage.h"
#include <limits>

namespace tlfea::contact::self_contact_filters {
Report MakeLayout(Limits limits, std::size_t owner_bytes, Layout& output) noexcept {
  if (!limits.max_facets || !limits.max_pairs || !limits.max_device_bytes ||
      !limits.max_host_bytes || limits.max_facets > UINT32_MAX ||
      limits.max_pairs > UINT32_MAX)
    return {Status::InvalidInput, "Filter batch limits are invalid"};
  Layout next;
  tl::util::BoundedArenaLayout device(limits.max_device_bytes);
  if (!device.Append<TriangleGeometry>(limits.max_facets, next.accepted) ||
      !device.Append<TriangleGeometry>(limits.max_facets, next.prepared) ||
      !device.Append<FacetProperties>(limits.max_facets, next.properties) ||
      !device.Append<FixedTrianglePair>(limits.max_pairs, next.pairs) ||
      !device.Append<PairResult>(limits.max_pairs, next.results))
    return {Status::ResourceLimit, "Filter batch device payload exceeds cap"};
  tl::util::BoundedArenaLayout host(limits.max_host_bytes);
  if (!host.Append<PairResult>(limits.max_pairs, next.host_results) ||
      owner_bytes > limits.max_host_bytes - host.bytes())
    return {Status::ResourceLimit, "Filter batch host payload exceeds cap"};
  next.forecast.facets = limits.max_facets;
  next.forecast.pairs = limits.max_pairs;
  next.forecast.device_bytes = device.bytes();
  next.forecast.owned_host_bytes = owner_bytes + host.bytes();
  // The only startup locals with size proportional to this module's forecast
  // are two layouts and one preflight value. CUDA allocator bookkeeping remains
  // under the enclosing process/device guard, as in the existing broadphase.
  constexpr std::size_t startup_fixed = 2 * sizeof(Layout) + sizeof(Preflight);
  if (startup_fixed > limits.max_host_bytes - next.forecast.owned_host_bytes)
    return {Status::ResourceLimit, "Filter batch startup host payload exceeds cap"};
  next.forecast.startup_host_bytes = next.forecast.owned_host_bytes + startup_fixed;
  next.forecast.device_allocations = 1;
  output = next;
  return {};
}
Preflight Batch::PreflightLimits(Limits limits) noexcept {
  Layout layout;
  Preflight result;
  result.report = MakeLayout(limits, sizeof(Batch) + sizeof(Impl), layout);
  if (result.report.status == Status::Ok) result.forecast = layout.forecast;
  return result;
}
}  // namespace tlfea::contact::self_contact_filters
