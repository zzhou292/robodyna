// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::source_gaps::detail {
struct Layout {
  tl::util::ArenaRegion secondary, main_nodes, mains, secondary_work, main_work, tags;
  Forecast forecast;
};
Report Prepare(const Input&, Limits, Layout&) noexcept;
bool Disjoint(const Input&, const Layout&, void*, std::size_t, Output) noexcept;
Report Rows(const Input&, unsigned char* tags) noexcept;
} // namespace tlfea::contact::radioss_type25::source_gaps::detail
