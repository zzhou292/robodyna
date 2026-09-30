// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../FENodalState.h"
#include <type_traits>

namespace tl::fea::cin_advance::groups {
// One record per immutable, disjoint group. Reused after the screen completes
// for motion status; neither phase publishes accepted state or a new clock.
struct Report {
  double minimum_dt;
  std::uint32_t first_node, last_node;
  NodalStatus status;
  bool visited, bounded;
};
static_assert(std::is_trivial<Report>::value);
static_assert(sizeof(Report) == 24);
inline constexpr unsigned Threads = 64;
} // namespace tl::fea::cin_advance::groups
