// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../FixedTriangleFeatureTypes.h"
#include "../SelfContactCurrentRegularity.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::current_regularity {

// Fixed-facet geometry differs by parent only through the retained native
// nodes.  The immutable template weights are copied once from the authenticated
// descriptor authority and then indexed directly by local facet.
struct FacetTemplate {
  double weights[3][4]{};
};

struct Layout {
  tl::util::ArenaRegion first_results;
  tl::util::ArenaRegion second_results;
  tl::util::ArenaRegion q4_templates;
  tl::util::ArenaRegion t3_templates;
  SelfContactCurrentRegularityForecast forecast;
};

struct Templates {
  FacetTemplate* q4 = nullptr;
  FacetTemplate* t3 = nullptr;
  std::size_t q4_count = 0;
  std::size_t t3_count = 0;
};

struct Storage {
  SelfContactCurrentParentResult* first_results = nullptr;
  SelfContactCurrentParentResult* second_results = nullptr;
  Templates templates;
};

SelfContactCurrentRegularityReport MakeLayout(
    const SelfContactActiveUseBinding&,
    SelfContactCurrentRegularityLimits, std::size_t implementation_bytes,
    Layout&) noexcept;
SelfContactCurrentRegularityReport ValidateSource(
    const SelfContactActiveUseBinding&,
    const SelfContactCurrentRegularityForecast&,
    Templates* = nullptr,
    FixedContactFacetReadCursor* = nullptr) noexcept;
SelfContactCurrentRegularityReport RunQuery(
    const SelfContactActiveUseBinding&,
    const SelfContactCurrentRegularityForecast&, VectorView,
    SelfContactActivityView, SelfContactCurrentParentResult* staging,
    const Templates&, FixedContactFacetReadCursor&,
    SelfContactCurrentRegularitySummary*) noexcept;

}  // namespace tlfea::contact::current_regularity

namespace tlfea::contact {

struct SelfContactCurrentRegularity::Impl {
  explicit Impl(const SelfContactActiveUseBinding& source) : binding(source) {}
  SelfContactActiveUseBinding binding;
  FixedContactFacetReadCursor facet_reader;
  tl::util::HostArena arena;
  current_regularity::Storage storage;
  SelfContactCurrentParentResult* publication = nullptr;
  SelfContactCurrentParentResult* staging = nullptr;
  SelfContactCurrentRegularityForecast forecast;
  SelfContactCurrentRegularitySummary summary;
  VectorView positions_identity;
  SelfContactActivityView activity_identity;
  std::uint64_t generation = 0;
  bool complete = false;
};

}  // namespace tlfea::contact
