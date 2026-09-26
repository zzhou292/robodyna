// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/source_nodal/Types.h"

namespace tlfea::contact::radioss_type25::source_nodal {
// Bounded host startup only. Two independent ordered += channels; no floating
// atomics, scalar coefficient formulas, sort, allocation, owner or solver clock.
// Supplied order/coverage/native quantities are caller obligations, not a
// scientific source assertion made by this value adapter.
Report Preflight(const Input&, Limits, Forecast&) noexcept;
// All counts, source rows and consumed ranges are admitted before staging.
// Scratch may be partial after arithmetic failure; public output remains exact.
// Failure-atomic publication does not imply atomic arithmetic or concurrency.
Report Accumulate(const Input&, Limits, void* scratch, std::size_t scratch_bytes,
                  Output) noexcept;
} // namespace tlfea::contact::radioss_type25::source_nodal
