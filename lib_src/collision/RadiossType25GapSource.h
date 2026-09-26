// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/source_gaps/Types.h"
namespace tlfea::contact::radioss_type25::source_gaps {
// Bounded host startup values in native units. The caller owns complete source
// populations, phase and NSV/MSR order. These arrays convey no runtime admission.
// Preflight validates counts/descriptors/profile only, without dereferencing rows.
Report Preflight(const Input&, Limits, Forecast&) noexcept;
// Output must be live typed storage. All writes occur after complete validation
// and arithmetic; scratch is temporary and disjoint from every input/output.
Report Build(const Input&, Limits, void* scratch, std::size_t bytes, Output) noexcept;
} // namespace tlfea::contact::radioss_type25::source_gaps
