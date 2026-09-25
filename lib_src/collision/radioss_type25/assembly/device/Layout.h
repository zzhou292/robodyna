// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../DeviceTypes.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::assembly::device_detail {
struct Layout {
  tl::util::ArenaRegion keys, sorted_keys, occurrences, offsets, failure, cub;
  IncidenceForecast forecast;
};
IncidenceStatus CheckLimits(IncidenceLimits) noexcept;
IncidenceStatus MakeLayout(IncidenceLimits, std::size_t scratch, std::size_t host,
    Layout&) noexcept;
} // namespace tlfea::contact::radioss_type25::assembly::device_detail
