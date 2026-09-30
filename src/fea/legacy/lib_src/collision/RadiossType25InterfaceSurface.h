// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/surface_interface/Types.h"
namespace tlfea::contact::radioss_type25::surface_interface {
// Descriptor/count-only preflight. The caller retains the complete source
// arrays and bounded HostArena storage; no allocation or source authentication.
Report Preflight(const Input&,Limits,Forecast&) noexcept;
// Complete selected IN24 classification and I25SURFI canonical filter.
// Requires supplied early reader order. Native winner observations are retained
// so the source owner can prove consumed order/ownership, not invent it.
// Rejection leaves output arena bytes and Snapshot unchanged.
Report Build(const Input&,Limits,tl::util::HostArena& output,
    tl::util::HostArena& scratch,Snapshot*) noexcept;
}
