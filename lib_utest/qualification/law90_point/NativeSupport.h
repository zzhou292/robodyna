#pragma once
#include "TestSupport.h"
#include "native/NativePoint.h"
#include "lib_utest/qualification/law90_preparation/native/NativeOracle.h"
#include <gtest/gtest.h>
#include <iomanip>

namespace law90_point_test {
struct NativeState {
  double history[10]{};
  int cursors[3]{};
  double values[11]{};
};
inline std::array<double, 33> NativePrepared(const law::PreparationInput& input,
                                          law::CurveView curve) {
  std::array<double, 33> values{};
  const auto parameters = law90_test::InputValues(input);
  const auto flags = law90_test::InputFlags(input);
  const int count = static_cast<int>(curve.count);
  law90_native_prepare(parameters.data(), flags.data(), curve.compression_strain,
                       curve.stress_pa, &count, values.data());
  return values;
}
inline void AdvanceNative(const double* prepared, law::CurveView curve,
                          const law::PointKinematics& input, double time, NativeState& native) {
  const int count = static_cast<int>(curve.count);
  law90_native_point(prepared, curve.compression_strain, curve.stress_pa, &count,
      input.total_b_minus_i_engineering, input.engineering_rate_s_inverse, &time,
      native.history, native.cursors, native.values);
}
inline ::testing::AssertionResult Compare(const double* actual, const std::uint32_t* cursors,
                                         const NativeState& native) {
  double expected[21];
  std::copy_n(native.history, 10, expected);
  std::copy_n(native.values, 11, expected+10);
  double stress_scale = std::max(1., native.history[0]);
  for (unsigned k = 0; k < 6; ++k) stress_scale = std::max(stress_scale, std::abs(native.values[k]));
  const double energy_scale = std::max({1., native.history[1], native.history[3], native.history[8]});
  for (unsigned k = 0; k < 21; ++k) {
    double scale = std::max(1., std::abs(expected[k]));
    if (k == 0 || (k >= 10 && k < 16)) scale = stress_scale;
    if (k == 1 || k == 3 || k == 8) scale = energy_scale;
    if (!std::isfinite(actual[k]) || !std::isfinite(expected[k]) ||
        std::abs(actual[k]-expected[k]) > 2e-10*scale)
      return ::testing::AssertionFailure() << "field " << k << std::setprecision(17)
          << " actual " << actual[k] << " native " << expected[k] << " bound " << 2e-10*scale;
  }
  for (unsigned k = 0; k < 3; ++k)
    if (cursors[k] != static_cast<std::uint32_t>(native.cursors[k]))
      return ::testing::AssertionFailure() << "cursor " << k << " actual " << cursors[k]
          << " native " << native.cursors[k];
  return ::testing::AssertionSuccess();
}
inline ::testing::AssertionResult Compare(const law::PointResult& result,
                                         const NativeState& native) {
  double actual[21];
  Pack(result, actual);
  return Compare(actual, result.history.cursor, native);
}
} // namespace law90_point_test
