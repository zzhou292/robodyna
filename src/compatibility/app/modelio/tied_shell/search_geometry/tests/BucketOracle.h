#pragma once
#include "../../TiedShellSearchGeometry.h"

namespace crash::modelio::tied_shell::test {
struct NativeBucketResult {
    std::vector<int> selected; // One-based IRECT rank, zero unmatched; NSV order.
    std::vector<std::array<double, 2>> st;
    std::vector<double> distance; // Original units; fresh unmatched DBL_MAX.
    std::vector<std::array<int, 2>> pairs; // Native (IRECT, NSV) order.
    std::array<double, 6> bounds{};
    std::array<int, 3> cells{};
};
// Test-only composition over the independently qualified immutable geometry.
// Native bucket enumeration is independent of the production GPU candidates.
NativeBucketResult NativeBucket(const TiedShellSearchGeometry&, std::size_t pair_capacity = 8388608);
} // namespace crash::modelio::tied_shell::test
