// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NativeOracle.h"
#include "../radioss_type25_normal/NativeOracle.h"
namespace type25_friction_test {
extern "C" void rd_type25_friction(const double*, const double*, const double*,
    const double*, double*, const int*);
n::NativeFrictionResult Oracle(const Case& c, bool foreign_slot) {
  n::NativeFrictionResult out;
  out.normal = type25_normal_test::Oracle(c.normal_config, c.input.normal, c.history.normal, foreign_slot);
  out.history.normal = out.normal.history;
  const double scalar[]{c.input.normal.penetration, c.input.normal.stiffness,
      c.input.normal.dt, c.input.dt12, out.normal.normal_force, c.controls.alpha, c.coefficients.base};
  const n::Vector vectors[]{c.input.normal_axis, c.input.relative_velocity,
      c.input.main_vertices[0], c.input.main_vertices[1], c.input.main_vertices[2], c.input.main_vertices[3]};
  double geometry[18];
  for (unsigned i = 0; i < 6; ++i) {
    geometry[3*i] = vectors[i].x; geometry[3*i+1] = vectors[i].y; geometry[3*i+2] = vectors[i].z;
  }
  const double history[]{c.history.previous_force.x, c.history.previous_force.y, c.history.previous_force.z,
      c.history.staged_force.x, c.history.staged_force.y, c.history.staged_force.z};
  const int slot = foreign_slot ? -1 : 1;
  double result[20]{};
  rd_type25_friction(scalar, geometry, c.coefficients.c, history, result, &slot);
  out.tangent_predictor = {result[0], result[1], result[2]};
  out.tangent_force = {result[3], result[4], result[5]};
  out.native_resultant = {result[6], result[7], result[8]};
  out.coefficient = result[9]; out.limiter = result[10]; out.contact_area = result[11];
  out.pressure = result[12]; out.friction_work = result[13];
  out.history.previous_force = {result[14], result[15], result[16]};
  out.history.staged_force = {result[17], result[18], result[19]};
  out.contact_active = c.input.normal.penetration != 0;
  return out;
}
} // namespace type25_friction_test
