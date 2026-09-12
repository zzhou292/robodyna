// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "SurfaceContactMass.h"

namespace tlfea::contact {
// Additive eight-node value profile; the legacy six-node NormalJacobian and
// BuildNormalJacobian contracts/layouts remain unchanged.
inline constexpr std::uint32_t MaxSurfaceMajorantNodes = 8;
enum class SurfaceMajorantStatus { Ok, InvalidInput, OutOfRange, InconsistentMask, Unrepresentable };
struct RepresentedJacobianTerm {
  std::uint32_t node = 0;
  Vec3 value;
  std::uint8_t translation_fixed_bits = 0; // Bits 1/2/4 fix world X/Y/Z.
};
struct SurfaceNodeMajorant {
  std::uint32_t node = 0;
  std::uint8_t translation_fixed_bits = 0;
  Vec3 jacobian; // Actual represented merge, then fixed components set to +0.
  double norm_upper = 0; // a_i >= Euclidean norm of this represented J_i.
  double diagonal_n_m = 0; // Outward k*a_i*sum_j(a_j), never a mass or timestep.
};
struct SurfaceJacobianMajorant {
  std::uint32_t count = 0; // Unique nodes, including canceled/fixed zero rows.
  SurfaceNodeMajorant nodes[MaxSurfaceMajorantNodes]{}; // Ascending node IDs.
  double stiffness_n_m = 0, norm_sum_upper = 0;
  bool valid = false;
};
} // namespace tlfea::contact
