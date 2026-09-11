// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"

namespace solid18_test {
struct NativePacket {
  std::array<double,348> values{};
  std::array<int,8> source_slot{};
  int status = -1;
};
NativePacket Native(const s::ReferenceInput& input);
// Unit-preserving comparison, with a component-group scale only for roundoff
// around zero. Source IDs, permutation, admission and signs are checked separately.
bool Agree(const std::array<double,348>& actual, const std::array<double,348>& expected,
           double relative = 2e-11);
std::array<double,348> NativeWorkingToSI(const std::array<double,348>& values);
}  // namespace solid18_test
