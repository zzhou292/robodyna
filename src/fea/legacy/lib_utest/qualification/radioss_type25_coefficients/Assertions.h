// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Cases.h"
#include <gtest/gtest.h>
#include <algorithm>
#include <cstdint>
#include <cstring>
namespace type25_coefficient_test {
inline void Number(double actual, double expected, bool exact = false) {
  ASSERT_TRUE(std::isfinite(actual)); ASSERT_TRUE(std::isfinite(expected));
  if (exact) {
    std::uint64_t a, b; std::memcpy(&a, &actual, 8); std::memcpy(&b, &expected, 8);
    EXPECT_EQ(a, b);
  } else {
    // Native pow and CUDA libdevice may round differently. No branch/result
    // classification tolerance is used; only the finite scalar value is compared.
    EXPECT_NEAR(actual, expected, 64 * std::numeric_limits<double>::epsilon() *
        std::max({std::abs(actual), std::abs(expected), 1e-300}));
  }
}
inline void Same(const Result& actual, const Result& expected, bool exact = false) {
  EXPECT_EQ(actual.status, expected.status);
  Number(actual.first, expected.first, exact); Number(actual.second, expected.second, exact);
}
} // namespace type25_coefficient_test
