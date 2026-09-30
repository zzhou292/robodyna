// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FacetFilters.h"
#include "Storage.h"

namespace tlfea::contact::self_contact_transaction {
FacetFilterForecast FacetFilters::Preflight(std::size_t facets, std::size_t pairs,
    std::size_t host_cap, std::size_t device_cap) noexcept {
  FacetFilterForecast result;
  const auto batch = filters::Batch::PreflightLimits({facets, pairs, device_cap, host_cap});
  result.report = batch.report;
  if (result.report.status != filters::Status::Ok) return result;
  tl::util::BoundedArenaLayout storage(host_cap);
  if (!storage.Append<filters::TriangleGeometry>(facets, result.storage.accepted) ||
      !storage.Append<filters::TriangleGeometry>(facets, result.storage.prepared) ||
      !storage.Append<filters::FacetProperties>(facets, result.storage.properties) ||
      !storage.Append<FixedTrianglePair>(pairs, result.storage.packed_pairs) ||
      !storage.Append<std::uint32_t>(pairs, result.storage.original_to_packed)) {
    result.report = {filters::Status::ResourceLimit, "Compact facet-filter storage exceeds host cap"};
    return result;
  }
  result.storage.bytes = storage.bytes();
  result.batch = batch.forecast;
  if (result.batch.owned_host_bytes < sizeof(filters::Batch) ||
      result.batch.startup_host_bytes < result.batch.owned_host_bytes) {
    result.report = {filters::Status::InvalidInput, "Facet-filter forecast is smaller than its handle"};
    return result;
  }
  tl::util::BoundedArenaLayout total(host_cap);
  tl::util::ArenaRegion region;
  if (!total.Append<std::byte>(sizeof(FacetFilters), region) ||
      !total.Append<std::byte>(storage.bytes(), region) ||
      !total.Append<std::byte>(result.batch.owned_host_bytes - sizeof(filters::Batch), region)) {
    result.report = {filters::Status::ResourceLimit, "Complete facet-filter owner exceeds host cap"};
    return result;
  }
  result.owned_host_bytes = total.bytes();
  if (!total.Append<std::byte>(result.batch.startup_host_bytes-result.batch.owned_host_bytes, region) ||
      !total.Append<std::byte>(2*sizeof(FacetFilterForecast) + 2*sizeof(tl::util::BoundedArenaLayout) +
          sizeof(tl::util::ArenaRegion) + sizeof(filters::Preflight), region)) {
    result.report = {filters::Status::ResourceLimit, "Facet-filter startup exceeds host cap"};
    return result;
  }
  result.startup_host_bytes = total.bytes();
  result.device_bytes = result.batch.device_bytes;
  result.device_allocations = result.batch.device_allocations;
  return result;
}

std::size_t PackCandidateLinearPairs(const FixedTrianglePair* pairs, std::size_t count,
    const MotionSupport* motion, const SelfContactSweptParentBounds* bounds,
    std::size_t facets, std::size_t parents,
    FixedTrianglePair* packed, std::uint32_t* original_to_packed) noexcept {
  if (!count) return 0;
  if (!pairs || !motion || !bounds || !packed || !original_to_packed ||
      count > UINT32_MAX) return SIZE_MAX;
  std::size_t write = 0;
  for (std::size_t ordinal = 0; ordinal < count; ++ordinal) {
    original_to_packed[ordinal] = UINT32_MAX;
    const auto pair = pairs[ordinal];
    if (pair.first >= facets || pair.second >= facets || pair.first == pair.second)
      continue;
    if (motion[pair.first].parent >= parents || motion[pair.second].parent >= parents)
      continue;
    if (ClassifyCandidatePairMotion(motion[pair.first], bounds[pair.first],
            motion[pair.second], bounds[pair.second]) != PairMotionAction::LinearNodalV1)
      continue;
    original_to_packed[ordinal] = static_cast<std::uint32_t>(write);
    packed[write++] = pair;
  }
  return write;
}

std::size_t LinearFacetSpanEnd(const FixedTrianglePair* pairs, std::size_t count,
    std::size_t first, const MotionSupport* motion, const SelfContactSweptParentBounds* bounds,
    std::size_t facets, std::size_t parents) noexcept {
  if (!pairs || !motion || !bounds || first >= count) return first;
  auto end = first;
  for (; end < count; ++end) {
    const auto pair = pairs[end];
    if (pair.first >= facets || pair.second >= facets || pair.first == pair.second ||
        motion[pair.first].parent >= parents || motion[pair.second].parent >= parents)
      break;
    if (ClassifyCandidatePairMotion(motion[pair.first], bounds[pair.first],
            motion[pair.second], bounds[pair.second]) != PairMotionAction::LinearNodalV1)
      break;
  }
  return end;
}

SelfContactTransactionReport FacetFilterFailure(const filters::Report& value,
    std::size_t pair) noexcept {
  SelfContactTransactionReport result;
  result.status = value.status == filters::Status::ResourceLimit
      ? SelfContactTransactionStatus::ResourceLimit
      : SelfContactTransactionStatus::FacetFilterFailure;
  result.filter_status = value.status;
  result.message = value.message;
  result.pair = pair;
  return result;
}
}  // namespace tlfea::contact::self_contact_transaction
