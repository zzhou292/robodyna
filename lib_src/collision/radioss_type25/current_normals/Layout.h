// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Stages.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::current_normals::detail {
struct Layout {
  tl::util::ArenaRegion normal,neighbor,eligible,tage,slots,references;
  Forecast forecast;
};
Report MakeLayout(const Input&,Limits,Layout&) noexcept;
Report Plan(const Input&,Limits,Layout&,double& length) noexcept;
Report Execute(const Input&,const Layout&,double length,void* scratch,Output) noexcept;
Work Construct(void* scratch,const Layout&) noexcept;
bool InputDisjoint(const Input&,const void*,std::size_t) noexcept;
Report Storage(const Input&,const Layout&,void* scratch,std::size_t bytes,Output) noexcept;
} // namespace tlfea::contact::radioss_type25::current_normals::detail
