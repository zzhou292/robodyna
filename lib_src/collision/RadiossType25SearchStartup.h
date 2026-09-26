// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/search_startup/Types.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::search_startup {
Forecast Preflight(std::size_t nodes, std::size_t primaries,
    std::size_t secondaries, Limits = {}) noexcept;
// Context-aware allocation forecast includes private auxiliary-ID validation.
Forecast Preflight(const Input&, Limits = {}) noexcept;
// Source reader's scalar multiplier rule uses native NUMNOD, including physical
// nodes outside the contact surface and any generated nonphysical primaries.
Status ResolveMultiplier(std::uint64_t native_model_nodes, double*) noexcept;
// Bounded host startup computation. Inputs and the immutable producer snapshot
// remain borrowed only during this call. Source completeness/fixed-main state
// belong to the source factory. No allocation or physical publication occurs.
// Every failure preserves all output bytes and the caller's view descriptor.
Report Build(const Input&, Limits, tl::util::HostArena& output,
    tl::util::HostArena& scratch, Snapshot*) noexcept;
// Complete ordinary TYPE25 geometric source with generated classical rigid
// primaries and no TYPE2/CIN contributors. Rigid membership adds no blanket
// TYPE25 same-body pair filter. Actual source authority belongs to the caller.
Report BuildRigidOnly(const Input&, Limits, tl::util::HostArena& output,
    tl::util::HostArena& scratch, Snapshot*) noexcept;
// Complete geometric stage, explicitly pending genuine tied augmentation.
// Complete declared contributor counts are retained, never zeroed to fit Build.
Report BuildGeometricBeforeTied(const Input&, Limits, tl::util::HostArena& output,
    tl::util::HostArena& scratch, GeometricSnapshot*) noexcept;
} // namespace tlfea::contact::radioss_type25::search_startup
