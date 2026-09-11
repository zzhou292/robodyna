// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law44/solid/Curve.h"

namespace tl::material::law44::solid {
namespace detail {
TL_LAW44_SOLID_HD inline double NativeInfinity() noexcept {
  return static_cast<double>(1e20f);  // CONSTANT_MOD default-REAL INFINITY.
}
TL_LAW44_SOLID_HD inline bool Positive(double x) noexcept {
  return tl::math::Finite(x) && x > 0;
}
TL_LAW44_SOLID_HD inline bool Coefficients(Material m, Parameters& p) noexcept {
  if (!Positive(m.young_pa) || !Positive(m.density_kg_m3) ||
      !tl::math::Finite(m.poisson_ratio) || m.poisson_ratio < 0 || m.poisson_ratio >= .5 ||
      !Positive(m.rate_c_per_s) || !Positive(m.rate_p) || !Positive(m.cutoff_hz)) return false;
  double stress_scale = 0;
  switch (m.native_units) {
    case WorkingUnits::SI: stress_scale = 1; break;
    case WorkingUnits::TonneMillimetreSecond: stress_scale = 1e6; break;
    default: return false;
  }
  p.material = m;
  // HM_READ_MAT44 order; the solid sound speed differs from its PM27 estimate.
  p.bulk_pa = m.young_pa / 3 / (1 - 2 * m.poisson_ratio);
  p.shear_pa = m.young_pa / 2 / (1 + m.poisson_ratio);
  p.twice_shear_pa = 2 * p.shear_pa;
  p.three_shear_pa = 3 * p.shear_pa;
  p.sound_speed_m_s = ::sqrt((p.bulk_pa + 4 * p.shear_pa / 3) / m.density_kg_m3);
  p.inverse_rate_c = 1 / m.rate_c_per_s;
  p.inverse_rate_p = 1 / m.rate_p;
  p.angular_cutoff_per_s = 2 * ::atan2(0., -1.) * m.cutoff_hz;
  p.stress_limit_pa = NativeInfinity() * stress_scale;
  p.stress_floor_pa = 1e-20 * stress_scale;
  return Positive(p.bulk_pa) && Positive(p.shear_pa) && Positive(p.twice_shear_pa) &&
      Positive(p.three_shear_pa) && Positive(p.sound_speed_m_s) &&
      Positive(p.inverse_rate_c) && Positive(p.inverse_rate_p) &&
      Positive(p.angular_cutoff_per_s) && Positive(p.stress_limit_pa) && Positive(p.stress_floor_pa);
}
TL_LAW44_SOLID_HD inline bool ParametersValid(const Parameters& p) noexcept {
  Parameters expected{};
  return CurveShape(p.curve) && Coefficients(p.material, expected) &&
      p.bulk_pa == expected.bulk_pa && p.shear_pa == expected.shear_pa &&
      p.twice_shear_pa == expected.twice_shear_pa && p.three_shear_pa == expected.three_shear_pa &&
      p.sound_speed_m_s == expected.sound_speed_m_s && p.inverse_rate_c == expected.inverse_rate_c &&
      p.inverse_rate_p == expected.inverse_rate_p &&
      p.angular_cutoff_per_s == expected.angular_cutoff_per_s &&
      p.stress_limit_pa == expected.stress_limit_pa && p.stress_floor_pa == expected.stress_floor_pa;
}
}  // namespace detail
TL_LAW44_SOLID_HD inline Status Prepare(Material material, Curve curve,
                                       Parameters& output) noexcept {
  Parameters p{};
  if (!detail::Coefficients(material, p)) return Status::InvalidParameters;
  if (!detail::CurveValid(curve)) return Status::InvalidCurve;
  p.curve = curve;
  output = p;
  return Status::Ok;
}
}  // namespace tl::material::law44::solid
