// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FacetFilters.h"
#include "AcceptedFacetFiltering.h"
#include "../self_contact_filters/Environment.h"

namespace tlfea::contact::self_contact_transaction {
SelfContactTransactionReport FacetFilters::AcceptedPairs(FixedTrianglePair* pairs,
    std::size_t* count) noexcept {
  if (device_failure_.status == filters::Status::DeviceFailure) return FacetFilterFailure(device_failure_);
  if (phase_ != Phase::Accepted || !source_ || !base_input_ || !motion_)
    return FacetFilterFailure({filters::Status::NoScene, "Facet-filter accepted scene is not current"});
  if (!device_scene_ || !filters::CompatibleHostArithmetic())
    return FilterAcceptedFacetPairs(*source_, base_input_, motion_, facets_, pairs, count);
  if (!pairs || !count || *count > pairs_ || !OutputDisjoint(count, sizeof(*count)) ||
      !OutputDisjoint(pairs, *count*sizeof(*pairs)))
    return FacetFilterFailure({filters::Status::InvalidInput, "Facet-filter accepted chunk aliases storage or exceeds capacity"});
  const auto uses = source_->facet_uses(); const auto parents = source_->parents();
  // Do not let a later invalid ordinal/parent preempt an earlier numeric error.
  // Evaluate only the valid metadata prefix; the shared serial fold below owns
  // every original error and mutation, including the first invalid suffix row.
  std::size_t prefix = 0;
  for (; prefix < *count; ++prefix) {
    const auto pair = pairs[prefix];
    if (pair.first >= facets_ || pair.second >= facets_ || pair.first == pair.second ||
        uses[pair.first].parent >= parents.size() || uses[pair.second].parent >= parents.size())
      break;
  }
  filters::ResultView view;
  if (prefix) {
    const auto report = batch_.Accepted({pairs, prefix}, stream_);
    if (report.status == filters::Status::UnsupportedEnvironment)
      return FilterAcceptedFacetPairs(*source_, base_input_, motion_, facets_, pairs, count);
    if (report.status != filters::Status::Ok) { ObserveFailure(report); Discard(); return FacetFilterFailure(report); }
    view = batch_.results();
    if (!view.complete || view.count != prefix || view.scene_generation != scene_generation_) {
      Discard(); return FacetFilterFailure({filters::Status::InvalidInput, "Facet-filter accepted publication differs from the current scene"});
    }
  }
  std::size_t ordinal = 0;
  const auto classified = [&](const CurrentFixedTriangle&, double, std::uint32_t,
                              const CurrentFixedTriangle&, double, std::uint32_t) {
    // Exactly one call per metadata-valid row, before compaction of that row.
    if (ordinal >= view.count)
      return SelfContactFacetFilterResult{SelfContactFacetFilterStatus::InvalidInput,
                                         SelfContactFacetFilterCategory::ExactRemaining};
    const auto& value = view.data[ordinal++];
    return SelfContactFacetFilterResult{value.status, value.category};
  };
  const auto result = detail::FilterAcceptedFacetPairsWith(*source_, base_input_, motion_,
      facets_, pairs, count, classified);
  if (result.status != SelfContactTransactionStatus::Ok) Discard();
  return result;
}
}  // namespace tlfea::contact::self_contact_transaction
