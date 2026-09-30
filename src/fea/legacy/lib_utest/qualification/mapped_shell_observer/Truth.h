// SPDX-License-Identifier: MIT
#pragma once
#include <cstddef>
#include <cstdint>
#include <limits>

namespace mapped_observer_truth {
static_assert(__FLT128_MANT_DIG__ == 113, "Independent binary128 observer oracle");
using High = __float128;
inline High Abs(High value) { return value < 0 ? -value : value; }
struct TruthSum {
  High value = 0, absolute = 0;
  std::size_t count = 0;
  void Add(double term) {
    value += High(term);
    absolute += Abs(High(term));
    ++count;
  }
  High Bound(unsigned parents, unsigned blocks, unsigned slots = 4) const {
    const auto rounds = (parents + blocks * 128u - 1) / (blocks * 128u);
    // Three/four slot terms per visit, two fixed seven-level trees, at most
    // two summary loads and the original identity seed. QEPH keeps slots=4.
    const High depth = slots * rounds + 7 + (blocks + 128u - 1) / 128u + 7 + 1;
    const High du = depth / High(std::uint64_t{1} << 53);
    const High additions = slots * High(parents) + 2 * High(blocks) * 128u + 2 * 128u + 1;
    // Absolute sums handle cancellation; the additive term covers gradual
    // underflow. Factor2 also encloses the bounded binary128 oracle error.
    return 2 * (du * absolute + additions * High(std::numeric_limits<double>::denorm_min())) / (1 - du);
  }
};
inline double Dot3(const double* a, const double* b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}
} // namespace mapped_observer_truth
