// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Measurement.h"
namespace tl::fea::type13::batch_detail::measurement {
inline constexpr unsigned Threads=64;
// Block-local read staging only. No retained layout or per-step allocation.
struct Tile {
  double work[Threads][ChannelCount],increment[Threads][ChannelCount];
  double kick[Threads][2],drift[Threads][2],native_dt[Threads];
  std::uint8_t active[Threads],newly_failed[Threads];
  Status status[Threads];
  bool proceed;
};
static_assert(std::is_trivial_v<Tile>);
static_assert(sizeof(Tile)==9096);
}
