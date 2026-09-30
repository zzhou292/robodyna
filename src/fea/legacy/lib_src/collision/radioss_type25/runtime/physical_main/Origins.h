// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Index.h"
#include "../../startup/PostGapmTypes.h"
namespace tlfea::contact::radioss_type25::runtime_detail::physical_main {
bool Origin(const Index&,const startup::PrimaryFaceIdentity&,const Face&) noexcept;
bool Support(const Index&,const startup::PostGapmMainSupport&,const Face&) noexcept;
bool SolidSupport(const Index&,std::uint64_t,const Face&) noexcept;
} // namespace tlfea::contact::radioss_type25::runtime_detail::physical_main
