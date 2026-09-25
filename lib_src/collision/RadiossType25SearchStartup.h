// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/search_startup/Types.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::search_startup {
Forecast Preflight(std::size_t nodes, std::size_t primaries,
    std::size_t secondaries, Limits = {}) noexcept;
// Source reader's scalar multiplier rule. Whole physical model node count is
// required, including nodes outside the selected contact surface.
Status ResolveMultiplier(std::uint64_t physical_nodes, double*) noexcept;
// Bounded host startup computation. Inputs and the immutable producer snapshot
// remain borrowed only during this call. Source completeness/fixed-main state
// belong to the source factory. No allocation or physical publication occurs.
// Every failure preserves all output bytes and the caller's view descriptor.
Report Build(const Input&, Limits, tl::util::HostArena& output,
    tl::util::HostArena& scratch, Snapshot*) noexcept;
} // namespace tlfea::contact::radioss_type25::search_startup
