// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TestSupport.h"
#include "lib_utest/qualification/solid18_reference/NativeOracle.h"

namespace solid18_force_test {
// Test-only numeric ABI. No object padding or borrowed production pointers.
struct NativeState {
  std::array<double,160> point{};   // Eight independent native packets, 20 each.
  std::array<double,11> global{};
  std::array<double,21> saved{};
  std::array<int,8> source_slot{};
  double initial_center_volume = 0;
};
struct NativeResult {
  NativeState next;
  std::array<double,304> observation{};
  std::array<double,24> force{};    // Original source slots after ABI mapping.
  std::array<double,1111> geometry{};
  std::array<double,7> diagnostics{};
  int status = -1;
};
NativeState NativeInitial(const s::ReferenceInput& input);
NativeResult Native(const s::Material&, const NativeState&, const s::PrescribedInterval&);
} // namespace solid18_force_test
