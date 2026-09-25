// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include <type_traits>
namespace type25_normal_test {
extern "C" void rd_type25_normal(const int*, const double*, const normal::NativeNormalInput*,
    const normal::NativeNormalHistory*, double*, int*, const int*);
static_assert(std::is_standard_layout_v<normal::NativeNormalInput>);
static_assert(sizeof(normal::NativeNormalInput) == 15 * sizeof(double));
static_assert(sizeof(normal::NativeNormalHistory) == 5 * sizeof(double));
normal::NativeNormalResult Oracle(const normal::ResolvedNormalConfig& config,
    const normal::NativeNormalInput& input, const normal::NativeNormalHistory& history,
    bool foreign_slot) {
  const int controls[]{config.engine.kdtint, config.engine.idtmins, config.engine.idtmins_int};
  const int slot = foreign_slot ? -1 : 1;
  double values[18]{};
  int terms = 0;
  rd_type25_normal(controls, &config.damping_factor, &input, &history, values, &terms, &slot);
  normal::NativeNormalResult out;
  out.history = {values[0], values[1], values[2], values[3], values[4]};
  for (unsigned i = 0; i < 4; ++i) out.weights[i] = values[5 + i];
  out.force_stiffness = values[9]; out.stability_stiffness = values[10];
  out.normal_force = values[11]; out.elastic_energy = values[12];
  out.damping_force = values[13]; out.damping_work = values[14];
  out.damping_coefficient = values[15]; out.separate_elastic_stiffness = values[16];
  out.separate_friction_damping = values[17]; out.terms_valid = terms != 0;
  return out;
}
} // namespace type25_normal_test
