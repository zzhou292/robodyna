// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"

namespace solid18_test {
struct NativePacket {
  std::array<double,ValueCount> values{};
  std::array<int,8> source_slot{};
  int status = -1;
};
NativePacket Native(const s::ReferenceInput& input);
// Unit-preserving comparison, with a component-group scale only for roundoff
// around zero. Source IDs, permutation, admission and signs are checked separately.
bool Agree(const std::array<double,ValueCount>& actual, const std::array<double,ValueCount>& expected,
           double relative = 2e-11);
// Separate conversion comparison: Jacobians and higher modes combine coordinate differences.
// Coordinate roundoff is bounded against their same-dimensional group norm,
// including a small component formed by cancellation. Source/slot identity is exact.
bool AgreeWorkingUnits(const std::array<double,ValueCount>& actual,
                       const std::array<double,ValueCount>& expected,
                       double coordinate_conditioning);
std::array<double,ValueCount> NativeWorkingToSI(const std::array<double,ValueCount>& values);
}  // namespace solid18_test
