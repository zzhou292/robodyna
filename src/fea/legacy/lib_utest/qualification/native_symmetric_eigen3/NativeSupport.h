#pragma once
#include "TestSupport.h"
#include "native/NativeSpectrum.h"
#include <gtest/gtest.h>
#include <iomanip>

namespace spectrum_test {
static_assert(sizeof(std::array<double, 15>) == 15*sizeof(double));
inline std::vector<std::array<double, 15>> Native(const std::vector<Packet>& packets) {
  std::vector<std::array<double, 15>> values(packets.size());
  for (std::size_t first = 0; first < packets.size(); first += 129) {
    const int count = static_cast<int>(std::min<std::size_t>(129, packets.size()-first));
    double tensor[129][6]{}, rate[129][6]{};
    for (int i = 0; i < count; ++i) {
      std::copy_n(packets[first+i].tensor, 6, tensor[i]);
      std::copy_n(packets[first+i].rate, 6, rate[i]);
    }
    int status = -1;
    spectrum_native(&tensor[0][0], &rate[0][0], &count, values[first].data(), &status);
    if (status != 0) {
      ADD_FAILURE() << "native wrapper rejected valid batch " << first;
      return {};
    }
  }
  return values;
}
inline ::testing::AssertionResult Compare(const double (&actual)[15], const double* expected,
                                         const Packet& packet, double relative = 2e-12) {
  if (Agree(actual, expected, packet, relative)) return ::testing::AssertionSuccess();
  for (unsigned k = 0; k < 15; ++k) {
    const double scale = k < 3 ? TensorScale(packet) : k < 12 ? 1. : 10.;
    if (!std::isfinite(actual[k]) || !std::isfinite(expected[k]) ||
        std::abs(actual[k]-expected[k]) > relative*scale)
      return ::testing::AssertionFailure() << "field " << k << " actual " << std::setprecision(17)
          << actual[k] << " native " << expected[k] << " bound " << relative*scale;
  }
  return ::testing::AssertionFailure();
}
} // namespace spectrum_test
