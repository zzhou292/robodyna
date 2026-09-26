// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/source_nodal/CorrectionTypes.h"
namespace tlfea::contact::radioss_type25::source_nodal::correction {
// Native-storage-order EightSlot STIFINT_ICONTROL, then ordered TYPE24 tail.
// Order, material/property operands and complete source coverage belong to the
// source binder. Empty secondary span declares no applicable TYPE24 occurrences.
Report Preflight(const Input&, Limits, Forecast&) noexcept;
// Startup-only bounded host composition. Public output is unchanged on failure;
// scratch may be partial. No sorting, allocations or independent state owner.
Report Apply(const Input&, Limits, void* scratch, std::size_t scratch_bytes, Output) noexcept;
}
