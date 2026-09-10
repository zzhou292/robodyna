// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/elements/type25/Type25Types.h"
#include <array>

namespace type25_test {
inline std::array<double,55> EvaluationValues(const tl::fea::type25::Evaluation& e) {
  std::array<double,55> values{};std::size_t at=0;
  auto scalar=[&](double x){values.at(at++)=x;};
  auto vector=[&](tl::math::Vec3 v){scalar(v.x);scalar(v.y);scalar(v.z);};
  for(double x:e.frame.axes.v)scalar(x);
  for(double x:e.frame.midpoint_axes.v)scalar(x);
  scalar(e.frame.length_m);scalar(e.frame.midpoint_length_m);
  vector(e.history.transverse_axis);vector(e.history.displacement_m);vector(e.history.rotation_rad);
  vector(e.history.local_force_N);vector(e.history.local_couple_Nm);
  for(double x:e.history.internal_work_J)scalar(x);
  scalar(e.history.failure_criterion);
  for(const auto& endpoint:e.endpoints){vector(endpoint.force_N);vector(endpoint.couple_Nm);}
  scalar(e.critical_dt_s);scalar(e.translation_stiffness_N_per_m);scalar(e.rotation_stiffness_Nm_per_rad);
  return values;
}
} // namespace type25_test
