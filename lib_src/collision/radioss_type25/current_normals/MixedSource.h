// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Layout.h"
#include "../startup/PostGapmTypes.h"
namespace tlfea::contact::radioss_type25::current_normals::detail {
bool MixedSourceDisjoint(const startup::Snapshot&,const void*,std::size_t) noexcept;
Report PlanMixed(const Input&,const startup::Snapshot&,Limits,Layout&,double& length) noexcept;
}
