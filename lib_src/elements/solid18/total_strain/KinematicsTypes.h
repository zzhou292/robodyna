// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ReferenceTypes.h"
#include "lib_src/elements/solid18/Solid18ForceTypes.h"
namespace tl::fea::solid18::total_strain {
struct KinematicsInput {
  Vec3 position_m[8]{};          // Original source slots, world coordinates.
  Vec3 velocity_m_s[8]{};        // Actual supplied velocity; not displacement/dt.
  double dt_s = 0;              // Zero is native constructor geometry only.
};
struct PointKinematics {
  double world_displacement_gradient[9]{};    // Row-major F-I.
  double material_displacement_gradient[9]{}; // Current-frame F-I.
  double selected_left_cauchy_green_minus_identity[6]{}; // xx,yy,zz,xy,yz,xz; tensor shears.
  double engineering_rate_per_s[6]{};         // Independent native corrected rate.
};
struct Kinematics {
  CurrentGeometry geometry;
  Vec3 reference_displacement_m[8]{};  // Native SGCOOR3; eighth is zero.
  PointKinematics point[8]{};
};
struct KinematicsScratch {
  Kinematics staged;
  StartupGeometry current;
  Vec3 native_position_m[8]{};
  Vec3 native_velocity_m_s[8]{};
};
}  // namespace tl::fea::solid18::total_strain
