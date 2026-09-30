// SPDX-License-Identifier: AGPL-3.0-or-later
// Field conventions adapted from OpenRadioss (C) 2026 Siemens.
#pragma once
#include "T3Data.h"
namespace tl::fea::t3 {
struct PrescribedInterval {
  Vec3 position[3]{},velocity[3]{},angular_velocity[3]{};
  double base_time=0,dt=0;
  std::uint64_t sample_index=1;
};
struct Kinematics {
  Matrix3 frame;
  Vec3 local_position[3]{};
  double derivative[3]{}; // PX1/PY1/PY2 (m); distinct from startup zeros.
  // XX,YY,XY,YZ,ZX,KXX,KYY,KXY. Raw first five m^2/s, last three m/s.
  double raw_rate[8]{},normalized_rate[8]{},corrected_velocity_difference[3]{};
  double area=0,characteristic_length=0,area_scale=0;
  double base_time=0,position_time=0,velocity_time=0,dt=0;
  std::uint64_t sample_index=0;
  bool valid=false;
};
static_assert(sizeof(Kinematics)<1024,"Bounded T3 rate record");
} // namespace tl::fea::t3
