// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RadiossType25SurfaceSource.h"
#include <vector>
namespace type25_surface_source_test {
namespace s = tlfea::contact::radioss_type25::source_surfaces;
struct NativeResult {
    std::vector<s::Face> faces;
    // Observation: a native buffered solid face was emitted for this row.
    // The surface routines have no FLAG_ELEM_INTER25 argument to observe.
    std::vector<std::uint8_t> surface_solid_flags;
};
// Serial native COMMON storage; bounded to 256 nodes and 64 rows per family.
// Supplied tables are explicit early reader order. No source/order authority is
// created by this qualification adapter; all expected selection is native.
NativeResult Oracle(const s::Input&);
}
