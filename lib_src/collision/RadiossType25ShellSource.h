// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/source_shells/Types.h"
namespace tlfea::contact::radioss_type25::source_shells {
// Host startup values for the complete declared ordinary-shell physical model.
// No allocation, source parser, physical owner, solver clock or native linkage.
// Preflight inspects source rows. Unsupported other-family contributions reject
// via the explicit profile; model-file completeness remains the app's obligation.
Report Preflight(const Input&, Limits, Forecast&) noexcept;
// Explicit mixed-physical nodal seed; geometry/main admission is unchanged.
// Physical shells can contribute ET/count regardless of contact face role, but
// every requested primary remains separately bound to ordinary exterior scope.
// This overload does not authorize coated/internal primary coefficients.
Report Preflight(const Input&, NativeNodalSeedView, Limits, Forecast&) noexcept;
// Outputs publish together only after all fields succeed. Borrowed host scratch
// and output are disjoint from inputs and each other. No early partial writes.
Report Build(const Input&, Limits, void* scratch, std::size_t scratch_bytes, Output) noexcept;
Report Build(const Input&, NativeNodalSeedView, Limits, void* scratch,
             std::size_t scratch_bytes, Output) noexcept;
// Native I25INI_GAP_N distributes nodal gaps to every oriented main corner.
// Use this on the actual expanded connectivity, including repeated T3 slot4.
Report MainGaps(const NodeFields*, std::size_t, const std::uint32_t (&nodes)[4],
    MainGapFields* output) noexcept;
} // namespace tlfea::contact::radioss_type25::source_shells
