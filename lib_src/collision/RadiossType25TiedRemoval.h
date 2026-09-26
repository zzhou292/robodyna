// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "radioss_type25/tied_removal/Types.h"
#include "lib_utils/BoundedArena.h"
namespace tlfea::contact::radioss_type25::tied_removal {
// Descriptor/count admission and bounded forecast. No allocation or mutation of
// input state. This is a source value stage, not a completeness authority token.
Forecast Preflight(const Input&, Limits={}) noexcept;
// Complete PRE_I2 -> REMN_I2OP for the selected serial TYPE25/ILEV28 profile.
// Inputs borrow the genuine pending geometric stage and complete finalized
// TYPE2 roster. Output owns all arrays; every failure preserves its bytes and
// descriptor. Scratch is caller-private and may change on failure.
Report Build(const Input&, Limits, tl::util::HostArena& output,
    tl::util::HostArena& scratch, Snapshot*) noexcept;
} // namespace tlfea::contact::radioss_type25::tied_removal
