// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::source_shells::detail {
struct Layout {
  tl::util::ArenaRegion nodes, primaries, secondary, selected_shells;
  Forecast forecast;
};
Report Prepare(const Input&, NativeNodalSeedView, Limits, Layout&) noexcept;
bool SeparateStorage(const Input&, NativeNodalSeedView, const Layout&, void*, std::size_t, Output) noexcept;
} // namespace tlfea::contact::radioss_type25::source_shells::detail
