#pragma once
#include "lib_src/solvers/ExplicitStepStability.h"
#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>
#include <gtest/gtest.h>

namespace tl::fea::reset_test {
inline std::uint64_t Bits(double value) {
  std::uint64_t bits;
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
}
inline void SameRows(const stability::RowBounds& a, const stability::RowBounds& b) {
  EXPECT_EQ(a.node_count, b.node_count); EXPECT_EQ(a.capacity, b.capacity);
  EXPECT_EQ(a.base_epoch, b.base_epoch); EXPECT_EQ(a.attempt, b.attempt);
  EXPECT_EQ(a.initialized, b.initialized); EXPECT_EQ(a.valid, b.valid);
  EXPECT_EQ(a.sealed, b.sealed);
}
inline std::vector<double> Seed(std::uint32_t n) {
  std::vector<double> values(2 * (std::size_t(n) + 4));
  const double pattern[]{17., -0., std::numeric_limits<double>::quiet_NaN(),
      -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::denorm_min(), -21.};
  for (std::size_t i = 0; i < values.size(); ++i) values[i] = pattern[i % 6];
  return values;
}
inline stability::RowBounds Rows(double* base, std::uint32_t n, unsigned fault) {
  stability::RowBounds result{base + 1, base + n + 5, n, n + 2, 7, 9, true, true, true};
  if (fault == 1) result.stiffness = nullptr;
  if (fault == 2) result.damping = nullptr;
  if (fault == 3) result.damping = result.stiffness;
  if (fault == 4) result.node_count = 0;
  if (fault == 5) result.capacity = n - 1;
  if (fault == 6) result.base_epoch = 9;
  if (fault == 7) result.attempt = 10;
  if (fault == 8) result.attempt = 11;
  if (fault == 9) {
    result.initialized = false;
    result.base_epoch = result.attempt = UINT64_MAX;
  }
  if (fault == 10) { result.valid = false; result.sealed = false; }
  if (fault == 11) { result.capacity = 0; result.stiffness = nullptr; result.base_epoch = 9; }
  return result;
}
inline void SameBytes(const std::vector<double>& a, const std::vector<double>& b) {
  ASSERT_EQ(a.size(), b.size());
  EXPECT_EQ(std::memcmp(a.data(), b.data(), a.size() * sizeof(double)), 0);
}
inline void CheckCleared(const std::vector<double>& before, const std::vector<double>& after,
                         std::uint32_t n, bool success) {
  ASSERT_EQ(before.size(), after.size());
  for (std::size_t i = 0; i < before.size(); ++i) {
    const bool live = (i >= 1 && i < std::size_t(n) + 1) ||
                      (i >= std::size_t(n) + 5 && i < 2 * std::size_t(n) + 5);
    EXPECT_EQ(Bits(after[i]), success && live ? Bits(0.) : Bits(before[i])) << i;
  }
}
} // namespace tl::fea::reset_test
