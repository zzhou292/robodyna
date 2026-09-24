// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/collision/RepresentedIntervalCrossing.h"
#include <cmath>
#include <vector>
#include <initializer_list>
#include <stdexcept>

namespace native_gpu_test {
// Qualification routing fixture only. For the checked small integral/half
// coordinates used by these synthetic callers, addition and dyadic scaling are
// exact and preserve incidence/time patterns. This is not a transformation for
// arbitrary captured binary64 data. Every component becomes nonzero, with a
// genuine 900-exponent X versus Y/Z spread. Benchmark/production paths are untouched.
inline void RequireNonzeroWideStorage(
    std::vector<tlfea::contact::RepresentedTrianglePath>& paths) {
  // Check the entire synthetic premise before changing any input.
  for (const auto& path : paths)
    for (const auto& vertex : path.vertices)
      for (const auto point : vertex.endpoint)
        for (double coordinate : {point.x,point.y,point.z})
          if (!std::isfinite(coordinate) || coordinate < -4 || coordinate > 4 ||
              2*coordinate != std::floor(2*coordinate))
            throw std::invalid_argument("Wide storage fixture requires bounded integral/half coordinates");
  for (auto& path : paths)
    for (auto& vertex : path.vertices)
      for (auto& point : vertex.endpoint) {
        point.x = std::ldexp(point.x + 8, -900);
        point.y += 8;
        point.z += 8;
      }
}
}  // namespace native_gpu_test
