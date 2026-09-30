// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstdint>
#include <cstddef>
#include <type_traits>
namespace tl::fea::qbat::mapped::measurement {
inline constexpr unsigned Threads=64;
// Transient block-local read staging. No retained packet ABI or arena changes.
// The ordered leader reads the same source-parent/local/channel sequence.
struct Tile {
  double internal_work[Threads][2],internal_increment[Threads][2];
  double plastic_work[Threads],plastic_increment[Threads];
  double viscous_work[Threads],viscous_increment[Threads];
  double kick_operand[Threads][4],drift_operand[Threads][4];
  double area_ratio[Threads],thickness_ratio[Threads],native_dt[Threads],maximum_strain[Threads];
  std::uint8_t valid[Threads],active[Threads],newly_removed[Threads];
  double sum[10],minimum[3],maximum;
  std::size_t active_count,removed_count;
  unsigned invalid[2];
  bool proceed;
};
static_assert(std::is_trivial_v<Tile>);
static_assert(sizeof(Tile)==10576);
} // namespace tl::fea::qbat::mapped::measurement
