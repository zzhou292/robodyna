// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law36/Types.h"
#include "lib_src/materials/detail/VinterValue.h"
namespace tl::material::law36 {
namespace detail {
TL_LAW36_HD inline double NativeSentinel() noexcept {
  return static_cast<double>(1e20f);  // Native CONSTANT_MOD INFINITY.
}
TL_LAW36_HD inline bool Positive(double x) noexcept {
  return tl::math::Finite(x) && x > 0;
}
TL_LAW36_HD inline bool CurveShape(Curve c) noexcept {
  return c.plastic_strain && c.yield_stress_pa && c.count >= 2 && c.count <= 1024;
}
TL_LAW36_HD inline bool Elastic(double e, double nu, double rho,
                               Parameters& p) noexcept {
  if (!Positive(e) || !Positive(rho) || !tl::math::Finite(nu) ||
      nu < 0 || nu >= .5) return false;
  p.young_pa = e;
  p.poisson_ratio = nu;
  p.density_kg_m3 = rho;
  // HM_READ_MAT36: same expression order, distinct solid bulk and sound speed.
  p.shear_pa = .5 * e / (1 + nu);
  p.twice_shear_pa = 2 * p.shear_pa;
  p.three_shear_pa = 3 * p.shear_pa;
  p.bulk_pa = e / 3 / (1 - 2 * nu);
  p.sound_speed_m_s = ::sqrt((p.bulk_pa + (4.0 / 3.0) * p.shear_pa) / rho);
  return Positive(p.shear_pa) && Positive(p.twice_shear_pa) &&
         Positive(p.three_shear_pa) && Positive(p.bulk_pa) &&
         Positive(p.sound_speed_m_s);
}
TL_LAW36_HD inline bool ParametersValid(const Parameters& p) noexcept {
  Parameters expected{};
  return CurveShape(p.curve) && Elastic(p.young_pa, p.poisson_ratio,
      p.density_kg_m3, expected) &&
      p.shear_pa == expected.shear_pa &&
      p.twice_shear_pa == expected.twice_shear_pa &&
      p.three_shear_pa == expected.three_shear_pa &&
      p.bulk_pa == expected.bulk_pa &&
      p.sound_speed_m_s == expected.sound_speed_m_s;
}
}  // namespace detail
TL_LAW36_HD inline Status Prepare(double e, double nu, double rho,
                                 Curve curve, Parameters& output) noexcept {
  Parameters p{};
  if (!detail::Elastic(e, nu, rho, p)) return Status::InvalidParameters;
  if (!detail::CurveShape(curve) || curve.plastic_strain[0] != 0)
    return Status::InvalidCurve;
  for (std::uint32_t i = 0; i < curve.count; ++i) {
    if (!tl::math::Finite(curve.plastic_strain[i]) ||
        curve.plastic_strain[i] >= detail::NativeSentinel() ||
        !detail::Positive(curve.yield_stress_pa[i]) ||
        (i && (curve.plastic_strain[i] <= curve.plastic_strain[i - 1] ||
               curve.yield_stress_pa[i] < curve.yield_stress_pa[i - 1])))
      return Status::InvalidCurve;
    if (i) {
      double value = 0;
      double slope = 0;
      if (!tl::material::detail::VinterValue(curve, curve.plastic_strain[i], value, slope))
        return Status::InvalidCurve;
    }
  }
  p.curve = curve;
  output = p;
  return Status::Ok;
}
}  // namespace tl::material::law36
