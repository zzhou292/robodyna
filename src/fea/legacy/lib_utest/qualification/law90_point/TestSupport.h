#pragma once
#include "lib_src/materials/law90/Point.h"
#include "lib_utest/qualification/law90_preparation/TestSupport.h"
#include <limits>

namespace law90_point_test {
namespace law = tl::material::law90;
using law90_test::OriginalInput;
using law90_test::OriginalBlankHuInput;
using law90_test::OriginalBlankHuCurve;
using law90_test::OriginalCurve;
using law90_test::SameBits;
using law90_test::Bytes;
using law90_test::Close;
struct ToyCurve {
  double x[4]{0, .2, .6, 1};
  double y[4]{0, 2e6, 3e6, 8e6};
  law::CurveView view() const { return {x, y, 4}; }
};
inline law::PreparationInput ToyInput() {
  auto input = OriginalInput();
  input.card_young_pa = 5e6;
  input.tension_cutoff_pa = 1e6;
  input.reference_density_kg_m3 = 1000;
  return input;
}
inline law::PointKinematics Stretch(double a, double b, double c,
                                   double angle = 0, double tilt = 0) {
  const double ca = std::cos(angle), sa = std::sin(angle);
  const double ct = std::cos(tilt), st = std::sin(tilt);
  const double rotation[3][3]{{ca*ct, -sa, ca*st}, {sa*ct, ca, sa*st}, {-st, 0, ct}};
  const double diagonal[3]{a*a-1, b*b-1, c*c-1};
  double matrix[3][3]{};
  for (unsigned i = 0; i < 3; ++i)
    for (unsigned j = 0; j < 3; ++j)
      for (unsigned k = 0; k < 3; ++k)
        matrix[i][j] += rotation[i][k]*diagonal[k]*rotation[j][k];
  law::PointKinematics result;
  result.total_b_minus_i_engineering[0] = matrix[0][0];
  result.total_b_minus_i_engineering[1] = matrix[1][1];
  result.total_b_minus_i_engineering[2] = matrix[2][2];
  result.total_b_minus_i_engineering[3] = 2*matrix[0][1];
  result.total_b_minus_i_engineering[4] = 2*matrix[1][2];
  result.total_b_minus_i_engineering[5] = 2*matrix[2][0];
  const double rate[6]{1.3, -2.7, .8, 3.1, -.9, 1.7};
  std::copy_n(rate, 6, result.engineering_rate_s_inverse);
  return result;
}
inline law::PointKinematics Path(unsigned step, bool rotate) {
  const double phase = static_cast<double>(step % 160) / 160;
  const double wave = .5*(1-std::cos(phase*2*std::acos(-1.)));
  const double tension = step >= 160 ? .65 : 0;
  auto input = Stretch(1-.72*wave+tension, 1-.31*wave, 1-.12*wave,
                       rotate ? .2+.013*step : 0, rotate ? -.4+.007*step : 0);
  for (unsigned k = 0; k < 6; ++k)
    input.engineering_rate_s_inverse[k] *= 1 + .2*std::sin(.09*step+k);
  return input;
}
TL_LAW90_HD inline void PackHistory(const law::PointHistory& h, double* values) {
  const double data[]{h.stress_norm_pa, h.maximum_path_energy_pa, h.scalar_rate_s_inverse,
      h.path_energy_pa, h.reserved5, h.strain_norm, h.unloading_factor,
      h.effective_modulus_pa, h.instantaneous_quasistatic_energy_pa, h.residual_strain};
  for (unsigned k = 0; k < 10; ++k) values[k] = data[k];
}
TL_LAW90_HD inline void Pack(const law::PointResult& result, double* values) {
  PackHistory(result.history, values);
  for (unsigned k = 0; k < 6; ++k) values[10+k] = result.cauchy_stress_pa[k];
  values[16] = result.sound_speed_m_s;
  values[17] = result.scalar_rate_s_inverse;
  values[18] = result.tangent_factor;
  values[19] = result.maximum_viscosity_pa_s;
  values[20] = result.active;
}
} // namespace law90_point_test
