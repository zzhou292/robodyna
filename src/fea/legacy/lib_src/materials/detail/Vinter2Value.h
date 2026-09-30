// SPDX-License-Identifier: AGPL-3.0-or-later
// VINTER2, OpenRadioss (C) 2026 Siemens. Native bidirectional cursor semantics.
#pragma once
#include "lib_src/math/Quaternion.h"
#include <cstdint>

#if defined(__CUDACC__)
#define TL_VINTER2_HD __host__ __device__
#else
#define TL_VINTER2_HD
#endif

namespace tl::material::detail {
struct Vinter2Result {
  double value = 0;
  double slope = 0;
  std::uint32_t cursor = 0;
};

// Borrowed, strictly increasing finite abscissas; cursor is the zero-based
// native IPOS. The caller owns immutable storage. Exact knots retain the
// incoming adjacent segment. Endpoints extrapolate without clamping Y.
TL_VINTER2_HD inline bool Vinter2Value(
    const double* x, const double* y, std::uint32_t count, double query,
    std::uint32_t cursor, Vinter2Result& output) noexcept {
  if (!x || !y || count < 2 || count > 1024 || cursor >= count - 1 ||
      !tl::math::Finite(query)) {
    return false;
  }
  for (std::uint32_t i = 0; i < count; ++i) {
    if (!tl::math::Finite(x[i]) || !tl::math::Finite(y[i]) ||
        (i && x[i] <= x[i - 1])) {
      return false;
    }
  }

  // SIGEPS90 computes ILEN from the incoming cursor, before VINTER2's loop.
  const std::uint32_t length = count - 1 - cursor;
  std::uint32_t iteration = 0;
  bool moved = true;
  while (moved) {
    ++iteration;
    moved = false;
    bool forward = iteration <= length - 1;
    if (forward) {
      forward = query > x[cursor + 1];
    }
    bool backward = cursor >= 1;
    if (backward) {
      backward = query < x[cursor];
    }
    if (forward) {
      ++cursor;
      moved = true;
    } else if (backward) {
      --cursor;
      moved = true;
    }
  }

  const double y2 = y[cursor + 1];
  const double y1 = y[cursor];
  const double x2 = x[cursor + 1];
  const double x1 = x[cursor];
  const double width = x2 - x1;
  const double rise = y2 - y1;
  if (!tl::math::Finite(width) || !tl::math::Finite(rise)) {
    return false;
  }
  const double slope = rise / width;
  const double value = y1 + slope * (query - x1);
  if (!tl::math::Finite(slope) || !tl::math::Finite(value)) {
    return false;
  }
  output = {value, slope, cursor};
  return true;
}
}  // namespace tl::material::detail
#undef TL_VINTER2_HD
