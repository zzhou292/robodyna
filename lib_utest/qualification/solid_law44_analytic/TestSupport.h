// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_utest/qualification/solid_law44_point/TestSupport.h"
#include <cstring>
#include <limits>

namespace law44_analytic_test {
namespace law = tl::material::law44::solid;
using law44_solid_test::Motion;
using law44_solid_test::Same;
using law44_solid_test::Values;
inline bool Bits(double a, double b) { return std::memcmp(&a, &b, sizeof(a)) == 0; }
inline law::Parameters Airbag(law::WorkingUnits units = law::WorkingUnits::TonneMillimetreSecond) {
  law::Material material{1000e6, .3, 1.95e-9 * 1e12, 8000, 8, 10000, units};
  const double scale = units == law::WorkingUnits::SI ? 1 : 1e6;
  law::Parameters result;
  EXPECT_EQ(law::PrepareMat024Analytic(material, 1000e6 / scale, 20e6 / scale,
      10e6 / scale, result), law::Status::Ok);
  return result;
}
inline law::Parameters Capped(double exponent = 1) {
  auto result = Airbag();
  EXPECT_EQ(law::PrepareAnalytic(result.material, {20e6, 10e6, exponent, 24e6, .8}, result),
      law::Status::Ok);
  return result;
}
inline void Empty(const law::Parameters& p) {
  EXPECT_EQ(p.material.hardening, law::HardeningKind::Analytic);
  EXPECT_EQ(p.curve.count, 0u);
  EXPECT_EQ(p.curve.plastic_strain, nullptr);
  EXPECT_EQ(p.curve.yield_stress_pa, nullptr);
}
} // namespace law44_analytic_test
