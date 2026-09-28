// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Measure.h"
namespace tl::fea::beam18::batch_detail::measurement {
inline constexpr unsigned Threads=64;
// Transient read staging; the existing endpoint work helper is unchanged.
struct Tile {
  double work[Threads][2],plastic[Threads],native_dt[Threads];
  int status[Threads];
  std::uint8_t valid[Threads];
  bool proceed;
};
static_assert(std::is_trivial_v<Tile>);
static_assert(sizeof(Tile)==2376);
}
