// SPDX-License-Identifier: AGPL-3.0-or-later
#include "FacetFilters.h"
#include "Storage.h"
#include "../self_contact_filters/Environment.h"

namespace tlfea::contact::self_contact_transaction {
namespace {
filters::Report Invalid(const char* message) noexcept {
  return {filters::Status::InvalidInput, message};
}
}

filters::Report FacetFilters::BeginCandidateChunk(
    const FixedTrianglePair* pairs, std::size_t count) noexcept {
  if (device_failure_.status == filters::Status::DeviceFailure) return device_failure_;
  if (!initialized_)
    return {filters::Status::NotInitialized, "Facet-filter adapter is not initialized"};
  if (phase_ != Phase::Candidate)
    return {filters::Status::NoScene, "Facet-filter candidate scene is not current"};
  // Inspect range/alias before any packing. Rejected replacement revokes the
  // previous borrow without reading or packing the rejected input range.
  const bool admitted = count <= pairs_ && (!count || pairs) &&
      OutputDisjoint(pairs, count * sizeof(*pairs));
  RevokeChunk();
  if (!admitted)
    return Invalid("Facet-filter candidate chunk is not a disjoint bounded borrow");
  chunk_ = pairs;
  chunk_count_ = count;
  packed_count_ = 0;
  chunk_ready_ = false;
  chunk_cpu_ = !device_scene_ || !filters::CompatibleHostArithmetic();
  if (chunk_cpu_) return {};
  packed_count_ = PackCandidateLinearPairs(pairs, count, motion_, bounds_,
      facets_, source_->parents().size(), packed_pairs_, original_to_packed_);
  if (packed_count_ == SIZE_MAX) {
    Discard();
    return Invalid("Facet-filter candidate packing storage is incomplete");
  }
  if (!packed_count_) {
    chunk_ready_ = true;
    return {};
  }
  const auto report = batch_.Linear({packed_pairs_, packed_count_},
      FacetPrismAxisLimit::VertexVertex, stream_);
  if (report.status == filters::Status::UnsupportedEnvironment) {
    // No command was issued. Do not requery this chunk after fenv restoration.
    chunk_cpu_ = true;
    return {};
  }
  if (report.status != filters::Status::Ok) {
    ObserveFailure(report);
    Discard();
    return report;
  }
  const auto view = batch_.results();
  if (!view.complete || view.count != packed_count_ ||
      view.scene_generation != scene_generation_) {
    Discard();
    return Invalid("Facet-filter compact publication differs from the current scene/chunk");
  }
  chunk_ready_ = true;
  return {};
}

FacetPrismReply FacetFilters::PrismAt(std::size_t ordinal) noexcept {
  FacetPrismReply result;
  result.supplied = true;
  if (device_failure_.status == filters::Status::DeviceFailure) {
    result.report = device_failure_;
    return result;
  }
  if (!initialized_) {
    result.report = {filters::Status::NotInitialized, "Facet-filter adapter is not initialized"};
    return result;
  }
  if (phase_ != Phase::Candidate) {
    result.report = {filters::Status::NoScene, "Facet-filter candidate scene is not current"};
    return result;
  }
  if (!chunk_ || ordinal >= chunk_count_ || chunk_count_ > pairs_ ||
      !OutputDisjoint(chunk_, chunk_count_ * sizeof(*chunk_))) {
    result.report = Invalid("Facet-filter candidate chunk is not a current disjoint borrow");
    Discard();
    return result;
  }
  const auto pair = chunk_[ordinal];
  if (pair.first >= facets_ || pair.second >= facets_ || pair.first == pair.second ||
      motion_[pair.first].parent >= source_->parents().size() ||
      motion_[pair.second].parent >= source_->parents().size()) {
    result.report = Invalid("Facet-filter candidate row has invalid facet or parent metadata");
    Discard();
    return result;
  }
  if (!filters::CompatibleHostArithmetic()) {
    chunk_cpu_ = true;
    chunk_ready_ = false;
  }
  if (chunk_cpu_) {
    result.supplied = false;
    return result;
  }
  if (!chunk_ready_) {
    result.report = Invalid("Facet-filter candidate chunk has no complete numerical query");
    Discard();
    return result;
  }
  const auto slot = original_to_packed_[ordinal];
  if (slot == UINT32_MAX || slot >= packed_count_ ||
      packed_pairs_[slot].first != pair.first || packed_pairs_[slot].second != pair.second) {
    result.report = Invalid("Facet-filter mapped result does not match the original pair");
    Discard();
    return result;
  }
  const auto view = batch_.results();
  if (!view.complete || view.count != packed_count_ ||
      view.scene_generation != scene_generation_) {
    result.report = Invalid("Facet-filter compact publication is stale");
    Discard();
    return result;
  }
  result.value = view.data[slot];
  return result;
}
}  // namespace tlfea::contact::self_contact_transaction
