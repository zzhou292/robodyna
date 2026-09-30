// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include <vector>
namespace qbat_force_test {
struct NativeState {
  std::array<double,116> state{};
  std::array<double,266> output{};
  explicit NativeState(const qb::HistoryValues& h):state(StateValues(h)) {}
  void Step(const Fixture&,const qb::PrescribedInterval&);
};
void CompareNative(const qb::ForceTrial&,const NativeState&);
} // namespace qbat_force_test
