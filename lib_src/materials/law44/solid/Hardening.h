// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Types.h"

namespace tl::material::law44::solid::detail {
TL_LAW44_SOLID_HD inline double NativeInfinity() noexcept {
  return static_cast<double>(1e20f);
}
TL_LAW44_SOLID_HD inline bool EmptyCurve(Curve c) noexcept {
  return !c.plastic_strain && !c.yield_stress_pa && c.count == 0;
}
TL_LAW44_SOLID_HD inline bool CanonicalZero(double x) noexcept {
  return x == 0 && ::copysign(1., x) > 0;
}
TL_LAW44_SOLID_HD inline bool EmptyAnalytic(const AnalyticHardening& a) noexcept {
  return CanonicalZero(a.a_pa) && CanonicalZero(a.b_pa) && CanonicalZero(a.exponent) &&
      CanonicalZero(a.maximum_stress_pa) && CanonicalZero(a.maximum_plastic_strain);
}
TL_LAW44_SOLID_HD inline bool NonnegativeHardening(double x) noexcept {
  return tl::math::Finite(x) && x >= 0;
}
// Exact selected HM_READ_MAT44 default and EPSGM operations in native units.
// Public coefficients are SI; the strain thresholds remain dimensionless.
TL_LAW44_SOLID_HD inline bool HardeningCoefficients(Material m, double scale,
    Parameters& p) noexcept {
  p.plastic_cap_strain = NativeInfinity();
  p.failure_plastic_strain = NativeInfinity();
  if (m.hardening == HardeningKind::Tabulated) return EmptyAnalytic(m.analytic);
  if (m.hardening != HardeningKind::Analytic) return false;
  const auto& a = m.analytic;
  if (!tl::math::Finite(a.a_pa) || a.a_pa <= 0 || !NonnegativeHardening(a.b_pa) ||
      !NonnegativeHardening(a.exponent) || !NonnegativeHardening(a.maximum_stress_pa) ||
      !NonnegativeHardening(a.maximum_plastic_strain)) return false;
  const double cap = a.maximum_stress_pa == 0 ? NativeInfinity() : a.maximum_stress_pa / scale;
  p.stress_limit_pa = cap * scale;
  if (a.maximum_plastic_strain != 0) p.failure_plastic_strain = a.maximum_plastic_strain;
  if (a.exponent != 0 && a.b_pa != 0) {
    p.plastic_cap_strain = ::pow((cap - a.a_pa / scale) / (a.b_pa / scale), 1 / a.exponent);
  }
  return tl::math::Finite(p.stress_limit_pa) && p.stress_limit_pa > 0 &&
      tl::math::Finite(p.plastic_cap_strain) && p.plastic_cap_strain >= 0;
}
} // namespace tl::material::law44::solid::detail
