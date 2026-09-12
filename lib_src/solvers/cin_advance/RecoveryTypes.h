// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../../constraints/tied_shell/runtime/CinMotionRows.h"

namespace tl::fea::cin_advance::recovery {
// A fresh, side-effect-free row result. Never a retained material or nodal state.
struct Row {
  constraints::tied_shell::SecondaryMotion motion;
  bool valid = false;
};
using FailureRow = unsigned int;
inline constexpr FailureRow NoFailure = UINT32_MAX;
static_assert(sizeof(FailureRow) == sizeof(std::uint32_t));
static_assert(sizeof(Row) == 104);
} // namespace tl::fea::cin_advance::recovery
