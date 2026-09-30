#pragma once
#include "lib_src/materials/failure/ShellConstantPlasticFailure.h"
#include <array>
#include <gtest/gtest.h>

namespace failure_test {
namespace f = tl::material::failure;
extern "C" void constant_failure_native(double, double, double, int, int,
    double, double, const double*, double*);
inline void CompareNative(const f::ConstantPlasticFailureParameters& p,
    const f::ConstantPlasticFailureHistory& base, const f::ConstantPlasticFailureInput& in,
    const f::ConstantPlasticFailureResult& actual, const std::array<double,5>& stress) {
  double values[3];
  constant_failure_native(p.failure_strain, base.damage, base.failure_time_s,
      base.point_active, in.element_active, in.plastic_strain_increment,
      in.native_evaluation_time_s, stress.data(), values);
  EXPECT_DOUBLE_EQ(actual.history.damage, values[0]);
  EXPECT_EQ(actual.history.failure_time_s, values[1]);
  EXPECT_EQ(actual.history.point_active, values[2] == 1);
  EXPECT_EQ(actual.failed_now, base.point_active && values[2] == 0);
}
} // namespace failure_test
