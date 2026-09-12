// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"

namespace tl::fea::solids::model_detail {
bool Same(const solid18::law44::Material& a, const solid18::law44::Material& b) noexcept {
  if (a.material.native_units != b.material.native_units) return false;
  if (!Same(a.material.young_pa, b.material.young_pa)) return false;
  if (!Same(a.material.poisson_ratio, b.material.poisson_ratio)) return false;
  if (!Same(a.material.density_kg_m3, b.material.density_kg_m3)) return false;
  if (!Same(a.material.rate_c_per_s, b.material.rate_c_per_s)) return false;
  if (!Same(a.material.rate_p, b.material.rate_p)) return false;
  if (!Same(a.material.cutoff_hz, b.material.cutoff_hz)) return false;
  if (a.material.hardening != b.material.hardening) return false;
  if (!Same(a.material.analytic.a_pa, b.material.analytic.a_pa)) return false;
  if (!Same(a.material.analytic.b_pa, b.material.analytic.b_pa)) return false;
  if (!Same(a.material.analytic.exponent, b.material.analytic.exponent)) return false;
  if (!Same(a.material.analytic.maximum_stress_pa, b.material.analytic.maximum_stress_pa)) return false;
  if (!Same(a.material.analytic.maximum_plastic_strain, b.material.analytic.maximum_plastic_strain)) return false;
  if (!Same(a.shear_pa, b.shear_pa)) return false;
  if (!Same(a.twice_shear_pa, b.twice_shear_pa)) return false;
  if (!Same(a.three_shear_pa, b.three_shear_pa)) return false;
  if (!Same(a.bulk_pa, b.bulk_pa)) return false;
  if (!Same(a.sound_speed_m_s, b.sound_speed_m_s)) return false;
  if (!Same(a.inverse_rate_c, b.inverse_rate_c)) return false;
  if (!Same(a.inverse_rate_p, b.inverse_rate_p)) return false;
  if (!Same(a.angular_cutoff_per_s, b.angular_cutoff_per_s)) return false;
  if (!Same(a.stress_limit_pa, b.stress_limit_pa)) return false;
  if (!Same(a.stress_floor_pa, b.stress_floor_pa)) return false;
  if (!Same(a.plastic_cap_strain, b.plastic_cap_strain)) return false;
  if (!Same(a.failure_plastic_strain, b.failure_plastic_strain)) return false;
  return SameCurves(Curve(a), Curve(b));
}
bool Same(const tl::material::law90::PreparedMaterial& a,
          const tl::material::law90::PreparedMaterial& b) noexcept {
  if (!a.initialized() || !b.initialized()) return false;
  const auto& x = a.reader();
  const auto& y = b.reader();
  if (!Same(x.density_kg_m3, y.density_kg_m3)) return false;
  if (!Same(x.reference_density_kg_m3, y.reference_density_kg_m3)) return false;
  if (!Same(x.card_young_pa, y.card_young_pa)) return false;
  if (!Same(x.initial_shear_pa, y.initial_shear_pa)) return false;
  if (!Same(x.poisson_ratio, y.poisson_ratio)) return false;
  if (!Same(x.contact_modulus_pa, y.contact_modulus_pa)) return false;
  if (!Same(x.contact_bulk_pa, y.contact_bulk_pa)) return false;
  if (!Same(x.tension_cutoff_pa, y.tension_cutoff_pa)) return false;
  if (!Same(x.hysteresis, y.hysteresis)) return false;
  if (!Same(x.shape, y.shape)) return false;
  if (!Same(x.alpha, y.alpha)) return false;
  if (!Same(x.curve_scale, y.curve_scale)) return false;
  if (!Same(x.curve_rate_s_inverse, y.curve_rate_s_inverse)) return false;
  if (!Same(x.filter_cutoff_hz, y.filter_cutoff_hz)) return false;
  if (x.smooth != y.smooth) return false;
  if (x.rate_flag != y.rate_flag) return false;
  if (x.loading_flag != y.loading_flag) return false;
  if (x.damage_flag != y.damage_flag) return false;
  if (x.tension_flag != y.tension_flag) return false;
  if (x.failure_mode != y.failure_mode) return false;
  if (x.material_viscosity_flag != y.material_viscosity_flag) return false;
  if (x.history_count != y.history_count) return false;
  if (x.cursor_count != y.cursor_count) return false;
  const auto& u = a.updated();
  const auto& v = b.updated();
  if (!Same(u.minimum_curve_slope_pa, v.minimum_curve_slope_pa)) return false;
  if (!Same(u.maximum_curve_slope_pa, v.maximum_curve_slope_pa)) return false;
  if (!Same(u.initial_curve_slope_pa, v.initial_curve_slope_pa)) return false;
  if (!Same(u.average_curve_slope_pa, v.average_curve_slope_pa)) return false;
  if (!Same(u.young_pa, v.young_pa)) return false;
  if (!Same(u.shear_pa, v.shear_pa)) return false;
  if (!Same(u.bulk_pa, v.bulk_pa)) return false;
  if (!Same(u.maximum_modulus_pa, v.maximum_modulus_pa)) return false;
  if (!Same(u.hourglass_modulus_pa, v.hourglass_modulus_pa)) return false;
  if (!Same(u.maximum_strain, v.maximum_strain)) return false;
  return SameCurves(Curve(a), Curve(b));
}
} // namespace tl::fea::solids::model_detail
