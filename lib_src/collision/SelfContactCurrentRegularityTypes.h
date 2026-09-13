// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "SelfContactActiveUseTypes.h"

#include <cstddef>
#include <cstdint>

namespace tlfea::contact {

class SelfContactActiveUseBinding;

enum class SelfContactCurrentRegularityStatus : std::uint8_t {
  Ok,
  AlreadyInitialized,
  NotInitialized,
  InvalidInput,
  IdentityMismatch,
  ResourceLimit,
  ParentGeometryUnresolved,
  FacetDegenerate,
  FacetOrientationMismatch,
  Unrepresentable,
  InvalidPair,
  ReceiptMismatch,
};

struct SelfContactCurrentRegularityReport {
  SelfContactCurrentRegularityStatus status =
      SelfContactCurrentRegularityStatus::Ok;
  std::size_t parent = SIZE_MAX;
  std::size_t facet = SIZE_MAX;
  std::size_t parents = 0;
  std::size_t facets = 0;
  const char* message = "OK";
};

enum class SelfContactCurrentParentState : std::uint8_t {
  Active,
  Removing,
  LongInactiveSkipped,
};

enum class SelfContactCurrentChartStatus : std::uint8_t {
  CertifiedQ4FixedDirection,
  CertifiedT3Native,
  SkippedLongInactive,
};

enum class SelfContactCurrentFacetStatus : std::uint8_t {
  Ok,
  InvalidInput,
  Degenerate,
  Reversed,
  Unrepresentable,
};

// A dimensionless scaled-Jacobian witness is used for numerical facet
// quality.  The acceptance boundary matches the checked triangle geometry
// primitive: strictly greater than 64*DBL_EPSILON.
struct SelfContactCurrentFacetWitness {
  Q4CertifiedIntegral double_area_m2;
  Q4IntegralInterval directed_chart_measure_m2;
  double scaled_jacobian_quality = 0;
  int exact_orientation_sign = 0;
};

struct SelfContactCurrentParentResult {
  std::uint64_t source_instance_id = 0;
  std::uint64_t source_eid = 0;
  std::size_t binding_parent = SIZE_MAX;
  std::size_t surface_parent = SIZE_MAX;
  unsigned arity = 0;
  unsigned level = 0;
  std::uint32_t facet_count = 0;
  std::uint32_t facets_evaluated = 0;
  SelfContactCurrentParentState state =
      SelfContactCurrentParentState::LongInactiveSkipped;
  SelfContactCurrentChartStatus chart =
      SelfContactCurrentChartStatus::SkippedLongInactive;
  bool geometry_evaluated = false;
  Vec3 chart_direction;
  Q4IntegralInterval current_area_enclosure_m2;
  FacetApproximationBound approximation;
  SelfContactCurrentFacetWitness minimum_area_witness;
  std::uint32_t minimum_area_local_facet = UINT32_MAX;
  double minimum_scaled_jacobian_quality = 0;
  std::uint32_t minimum_quality_local_facet = UINT32_MAX;
  Q4IntegralInterval minimum_directed_chart_measure_m2;
  std::uint32_t minimum_directed_local_facet = UINT32_MAX;
};

struct SelfContactCurrentRegularitySummary {
  std::uint64_t generation = 0;
  std::size_t parents = 0;
  std::size_t facets = 0;
  std::size_t facets_evaluated = 0;
  std::size_t certified_parents = 0;
  std::size_t active_parents = 0;
  std::size_t removing_parents = 0;
  std::size_t skipped_parents = 0;
  Q4IntegralInterval certified_current_area_enclosure_m2;
  double minimum_scaled_jacobian_quality = 0;
  std::size_t minimum_quality_parent = SIZE_MAX;
  std::uint32_t minimum_quality_local_facet = UINT32_MAX;
  double maximum_approximation_upper_m = 0;
  std::size_t maximum_approximation_parent = SIZE_MAX;
};

struct SelfContactCurrentRegularityView {
  const SelfContactCurrentParentResult* data = nullptr;
  std::size_t count = 0;
  bool complete = false;
  SelfContactCurrentRegularitySummary summary;
};

struct SelfContactCurrentRegularityLimits {
  std::size_t max_parents = 2048;
  std::size_t max_facets = 65536;
  std::size_t max_host_bytes = 128u << 20;
  static constexpr SelfContactCurrentRegularityLimits Vehicle() noexcept {
    return {524288, 16777216, 8ull << 30};
  }
};

struct SelfContactCurrentRegularityForecast {
  std::size_t parents = 0;
  std::size_t facets = 0;
  std::size_t publication_records = 0;
  std::size_t facet_staging_records = 0;
  std::size_t arena_bytes = 0;
  std::size_t retained_active_use_bytes = 0;
  std::size_t owned_payload_bytes = 0;
  std::size_t startup_payload_bytes = 0;
};

struct SelfContactCurrentRegularityPreflight {
  SelfContactCurrentRegularityReport report;
  SelfContactCurrentRegularityForecast forecast;
};

class SelfContactCurrentRegularity;

// Copyable authority value for one successful complete query.  Its fields are
// private so callers cannot construct a fresh generation or substitute input
// identities.  A failed query preserves both the prior publication and receipt.
class SelfContactCurrentRegularityReceipt {
 public:
  SelfContactCurrentRegularityReceipt() noexcept = default;
  std::uint64_t generation() const noexcept { return generation_; }
  bool prepared() const noexcept {
    return owner_identity_ != nullptr && generation_ != 0;
  }
  bool MatchesInputs(VectorView positions,
                     SelfContactActivityView activity) const noexcept {
    return positions_.data == positions.data &&
        positions_.node_count == positions.node_count &&
        positions_.node_stride == positions.node_stride &&
        positions_.component_stride == positions.component_stride &&
        activity_.base == activity.base &&
        activity_.current == activity.current &&
        activity_.parent_count == activity.parent_count;
  }

 private:
  friend class SelfContactCurrentRegularity;
  const void* owner_identity_ = nullptr;
  const SelfContactActiveUseBinding* binding_identity_ = nullptr;
  std::uint64_t generation_ = 0;
  VectorView positions_;
  SelfContactActivityView activity_;
};

}  // namespace tlfea::contact
