// SPDX-License-Identifier: MIT
#pragma once
#include "Fixture.h"
#include <vector>
namespace t3_one_point_test {
struct NativeState {
  std::array<double,37> state{};
  std::array<double,126> output{};
  explicit NativeState(const t3::OnePointHistory& initial):state(StateValues(initial.values())) {}
  int Step(const Fixture&,const t3::PrescribedInterval&);
};
void CompareNative(const t3::OnePointForceTrial&,const NativeState&);
inline void Close(double a,double b,double absolute=2e-13,double relative=2e-10) {
  EXPECT_TRUE(std::isfinite(a));
  EXPECT_TRUE(std::isfinite(b));
  EXPECT_NEAR(a,b,absolute+relative*std::max(std::abs(a),std::abs(b)));
}
} // namespace t3_one_point_test
