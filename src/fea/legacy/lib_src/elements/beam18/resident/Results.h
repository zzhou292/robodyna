// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Force.h"
#include <type_traits>

namespace tl::fea::beam18 {
// Public observations contain no borrowed device/reference/material pointers.
struct Result {
  HistoryValues history;
  ForceStamp stamp;
  ForceGeometry geometry;
  GeneralizedRate rate;
  point::Result point[4]{};
  Vec3 rhs_force_n[2]{}, rhs_couple_nm[2]{};
  ForceDiagnostics diagnostics;
};
static_assert(std::is_trivially_copyable_v<Result>);
struct ResultBuffer { Result* values = nullptr; std::size_t count = 0; };
} // namespace tl::fea::beam18
