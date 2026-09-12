// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../weighted_surface/Mapping.h"
#include "../SurfaceJacobianMajorantTypes.h"

namespace tlfea::contact {
enum class SurfacePenaltyStatus {
  Ok, InvalidInput, OutOfRange, NonFiniteResult, ZeroDistance, GapMismatch,
  InconsistentMask, Unrepresentable
};
struct SurfacePenaltyEndpoint {
  WeightedSurfacePoint point;
  std::uint8_t translation_fixed_bits[4]{}; // Per original slot: X/Y/Z = 1/2/4.
  double reference_half_thickness_m = 0; // Constant radius, no normal offset DOF.
};
struct SurfacePenaltyInput {
  VectorView positions;
  VectorView velocities;
  SurfacePenaltyEndpoint a;
  SurfacePenaltyEndpoint b;
  double declared_gap_m = 0;
  double stiffness_n_m = 0; // Explicit positive local coefficient; no area policy.
};
struct SurfacePenaltyNode {
  std::uint32_t node = 0;
  std::uint8_t translation_fixed_bits = 0;
  Vec3 force_n; // Full force, including reactions on fixed components.
  Vec3 free_force_n; // Only the declared free translation components.
};
struct SurfacePenaltyPacket {
  WeightedPointKinematics a;
  WeightedPointKinematics b;
  double distance_m = 0;
  double gap_m = 0;
  Vec3 normal; // Represented (x_A-x_B)/distance, no ideal unit-vector replacement.
  double normal_velocity_m_s = 0;
  double normal_force_n = 0;
  double elastic_energy_j = 0;
  Vec3 force_a_n;
  Vec3 force_b_n;
  WeightedNodalForces endpoint_a;
  WeightedNodalForces endpoint_b;
  std::uint32_t count = 0;
  SurfacePenaltyNode nodes[8]{}; // Source-order merge, then ascending node index.
  SurfaceJacobianMajorant normal_majorant;
  bool active = false;
  bool valid = false;
};
} // namespace tlfea::contact
