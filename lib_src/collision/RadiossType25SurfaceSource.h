// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/source_surfaces/Types.h"
namespace tlfea::contact::radioss_type25::source_surfaces {
// Descriptor-only, conservative complete-table capacity before borrowed reads.
Report Preflight(const Input&, Limits, Forecast&) noexcept;
// Exact selected initial CREATE_SURFACE_FROM_ELEMENT phase for provided native
// reader-order tables. Complete private staging precedes any output/descriptor
// publication. Source authority, I25SURFI role/filter/SH2, main K/gaps and later
// erosion are separate; success creates no runtime or physical receipt.
Report Build(const Input&, Limits, tl::util::HostArena& output,
    tl::util::HostArena& scratch, Snapshot*) noexcept;
}
