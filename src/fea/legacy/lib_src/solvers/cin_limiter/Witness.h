// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NodalCinStructuralLimit.h"

namespace tl::fea::cin_limiter {
struct Witness {
  NodalCinLimitValues values;
  std::uint64_t epoch = 0, attempt = 0;
};
} // namespace tl::fea::cin_limiter
