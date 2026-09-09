// SPDX-License-Identifier: AGPL-3.0-or-later
// QEPH conventions adapted from OpenRadioss, Copyright (C) 2026 Siemens.
// See the owning source manifest and LICENSE.md. No physical state owner.
#pragma once
#include "QephData.h"

namespace tl::fea::qeph {
struct PrescribedInterval {
  Vec3 position_endpoint[4]{}; // World m, at base_time+dt.
  Vec3 velocity_midpoint[4]{}; // World m/s, at base_time+dt/2.
  Vec3 omega_midpoint[4]{};    // World rad/s; no hidden quaternion state.
  double base_time=0,dt=0;
  std::uint64_t sample_index=0; // Caller label, not a solver epoch.
};

struct Kinematics {
  Matrix3 frame; // Row-major, current world basis columns.
  double area=0,reciprocal_area=0,characteristic_length=0;
  double nodal_factors[2]{}; // FACN, dimensionless; retained for later scatter.
  double raw_warpage_abs=0,effective_warpage=0; // m; before/after native switch.
  bool planar=false;
  Vec3 local_position[4]{}; // Centered COREL x/y with alternating effective Z1.
  Vec3 local_normals[4]{};  // Native VQN in local axes; flat branch reports +Z.
  // Native mixed translation/rotation projection coefficients. Their scaling
  // is source-coordinate dependent; not a general physical tensor inverse.
  double projection_inverse[6]{}; // DI: xx yy zz xy xz yz; flat branch zeros.
  Vec3 projection_columns[4]{};  // DB; flat branch zeros.
  double projected_omega[8]{};   // Two local components per node, rad/s.
  double regular_rate[8]{}; // XX YY XY XZ YZ KXX KYY KXY; first5 /s, last3 /(m*s).
  double hourglass_rate[6]{}; // First2/last2 m/s; components2/3 /s.
  double base_time=0,dt=0;
  std::uint64_t sample_index=0;
};
} // namespace tl::fea::qeph
