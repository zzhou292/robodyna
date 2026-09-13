// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once

#include "../FixedTriangleFeatureTypes.h"
#include "../SelfContactCurrentRegularity.h"
#include "lib_utils/BoundedArena.h"

namespace tlfea::contact::current_regularity {

struct Layout {
  tl::util::ArenaRegion first_results;
  tl::util::ArenaRegion second_results;
  tl::util::ArenaRegion facet_staging;
  SelfContactCurrentRegularityForecast forecast;
};

struct Storage {
  SelfContactCurrentParentResult* first_results = nullptr;
  SelfContactCurrentParentResult* second_results = nullptr;
  CurrentFixedTriangle* facet_staging = nullptr;
};

SelfContactCurrentRegularityReport MakeLayout(
    const SelfContactActiveUseBinding&,
    SelfContactCurrentRegularityLimits, std::size_t implementation_bytes,
    Layout&) noexcept;
SelfContactCurrentRegularityReport ValidateSource(
    const SelfContactActiveUseBinding&,
    const SelfContactCurrentRegularityForecast&) noexcept;
SelfContactCurrentRegularityReport RunQuery(
    const SelfContactActiveUseBinding&,
    const SelfContactCurrentRegularityForecast&, VectorView,
    SelfContactActivityView, SelfContactCurrentParentResult* staging,
    CurrentFixedTriangle* facet_staging,
    SelfContactCurrentRegularitySummary*) noexcept;

}  // namespace tlfea::contact::current_regularity

namespace tlfea::contact {

struct SelfContactCurrentRegularity::Impl {
  explicit Impl(const SelfContactActiveUseBinding& source) : binding(source) {}
  SelfContactActiveUseBinding binding;
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
