// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <stdexcept>
extern "C" void law44_solid_analytic_native(const double*, const double*, int,
    const double*, const double*, double, int, double*, double*, int*);
namespace law44_analytic_test {
law44_solid_test::NativeResult Native(const law::Parameters& p, const law::History& h,
    const law::Input& in, bool initialization) {
  if (p.material.hardening != law::HardeningKind::Analytic || !law::detail::EmptyCurve(p.curve) ||
      h.curve_cursor) throw std::invalid_argument("Analytic native packet shape");
  const auto& m = p.material;
  const auto& a = m.analytic;
  const double material[]{m.young_pa, m.poisson_ratio, m.density_kg_m3,
      m.rate_c_per_s, m.rate_p, m.cutoff_hz == 10000 ? 0 : m.cutoff_hz, a.a_pa};
  const double analytic[]{a.a_pa, a.b_pa, a.exponent, a.maximum_stress_pa, a.maximum_plastic_strain};
  double base[14]{}, motion[7]{}, values[19]{};
  std::copy_n(h.stress_pa, 6, base);
  std::copy_n(h.engineering_strain, 6, base + 6);
  base[12] = h.plastic_strain; base[13] = h.filtered_rate_per_s;
  std::copy_n(in.engineering_rate_per_s, 6, motion); motion[6] = in.dt_s;
  law44_solid_test::NativeResult result;
  int status = -1;
  law44_solid_analytic_native(material, analytic, static_cast<int>(m.native_units),
      base, motion, in.relative_density, initialization, values, result.prepared.data(), &status);
  if (status) throw std::runtime_error("Analytic native packet rejected");
  std::copy_n(values, 6, result.result.history.stress_pa);
  std::copy_n(values + 6, 6, result.result.history.engineering_strain);
  result.result.history.plastic_strain = values[12];
  result.result.history.filtered_rate_per_s = values[13];
  result.result.plastic_increment = values[14];
  result.result.yield_stress_pa = values[15];
  result.result.sound_speed_m_s = values[16];
  result.result.material_viscosity_pa_s = values[17];
  result.result.tangent_factor = values[18];
  return result;
}
}
