// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../NormalReuseQualification.h"
#include "../ExactPathReuseQualification.h"
#include "../CommonPointReuseQualification.h"
#include "../RelativeSeparationQualification.h"
namespace tlfea::contact::represented_interval_crossing::native {
using NormalCounters = represented_interval_crossing::NormalReuseCounters;
enum class NormalReuse { Recompute, Memoize };
enum class ExactPathReuse { Original, Optimized };
enum class CommonPointReuse { Original, Optimized };
using CommonPointCounters = represented_interval_crossing::CommonPointReuseCounters;
using ExactPathCounters = represented_interval_crossing::ExactPathReuseCounters;
enum class SeparationProof { LegacyAabb, RelativeFaces };
using ProjectionDomain = represented_interval_crossing::ExactProjectionDomain;
using SeparationCounters = represented_interval_crossing::RelativeSeparationCounters;

inline void CountSeparationOperation(
    SeparationCounters* counters, std::size_t SeparationCounters::* field) noexcept {
  if (!counters) return;
  auto& value = counters->*field;
  if (value == SIZE_MAX) counters->saturated = true;
  else ++value;
}

inline void CountNormalOperation(
    NormalCounters* counters, std::size_t NormalCounters::* field) noexcept {
  if (!counters) return;
  auto& value = counters->*field;
  if (value == SIZE_MAX) counters->saturated = true;
  else ++value;
}

inline void CountExactPathOperation(
    ExactPathCounters* counters, std::size_t ExactPathCounters::* field) noexcept {
  if (!counters) return;
  auto& value = counters->*field;
  if (value == SIZE_MAX) counters->saturated = true;
  else ++value;
}

inline void CountCommonPointOperation(
    CommonPointCounters* counters, std::size_t CommonPointCounters::* field) noexcept {
  if (!counters) return;
  auto& value = counters->*field;
  if (value == SIZE_MAX) counters->saturated = true;
  else ++value;
}

struct DyadicTime {
  std::uint64_t numerator = 0;
  unsigned depth = 0;
};

struct Cell {
  std::uint64_t lower = 0;
  std::uint64_t upper = 1;
  unsigned depth = 0;
};

inline DyadicTime Lower(const Cell& cell) { return {cell.lower, cell.depth}; }
inline DyadicTime Upper(const Cell& cell) { return {cell.upper, cell.depth}; }
inline DyadicTime Middle(const Cell& cell) {
  return {cell.lower + cell.upper, cell.depth + 1};
}


}  // namespace tlfea::contact::represented_interval_crossing::native
