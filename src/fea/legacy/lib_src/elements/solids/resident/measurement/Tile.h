// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstdint>
#include <type_traits>
namespace tl::fea::solids::batch_detail::measurement {
inline constexpr unsigned Threads=64;
// Five families reuse one transient tile, with at most eight local operands.
struct Tile {
  double work[Threads],hourglass_work[Threads],distortion_work[Threads],plastic_work[Threads],native_dt[Threads];
  double kick[Threads][8],drift[Threads][8];
  int status[Threads];
  std::uint8_t valid[Threads];
  double sum[6],minimum_dt;
  unsigned invalid[2];
  bool finite[6];
  bool proceed;
};
static_assert(std::is_trivial_v<Tile>);
static_assert(sizeof(Tile)==11144);
} // namespace tl::fea::solids::batch_detail::measurement
