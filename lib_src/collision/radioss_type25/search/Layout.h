// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Device.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::search::detail {
struct Layout {
  tl::util::ArenaRegion roles, reference[2], masks[2], gaps[2], partials, control;
  Forecast forecast;
};
Status CheckSource(const Source&,Limits) noexcept;
Status MakeLayout(const Source&,Limits,std::size_t owner_bytes,Layout&) noexcept;
Device Bind(void*,const Layout&,const Source&,const units_detail::Factors&) noexcept;
}
