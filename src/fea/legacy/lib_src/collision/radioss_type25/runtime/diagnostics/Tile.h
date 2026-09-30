// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include <cstdint>
#include <type_traits>
namespace tlfea::contact::radioss_type25::runtime_detail::diagnostics {
inline constexpr unsigned Threads=64;
// Fixed block-local read staging only; no retained device arena or history.
// Separate arrays keep neighboring lane stores adjacent in shared memory.
struct Tile {
  double elastic_energy[Threads],damping_work[Threads],friction_work[Threads];
  std::uint32_t positive[Threads],contact_active[Threads];
};
struct Operand {
  double elastic_energy,damping_work,friction_work;
  std::uint32_t positive,contact_active;
};
static_assert(std::is_trivial<Tile>::value);
static_assert(std::is_trivial<Operand>::value);
static_assert(sizeof(Tile)==2048);
static_assert(sizeof(Operand)==32);
} // namespace tlfea::contact::radioss_type25::runtime_detail::diagnostics
