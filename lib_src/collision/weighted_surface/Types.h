// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../SurfaceContactTypes.h"

namespace tlfea::contact {
// Frozen linear weights over original parent nodes, in original cyclic order.
// A physical facet may compose these from its fixed Q4 corner shapes. Such a
// composition is not the Q4 shape evaluated at averaged facet parameters.
// This value carries no source identity, geometry, owner, mass or clock.
struct WeightedSurfacePoint {
  std::uint32_t nodes[4]{};
  double weights[4]{};
  std::uint32_t count = 0; // Exactly three or four, including zero-weight slots.
};
struct WeightedPointKinematics {
  Vec3 position;
  Vec3 velocity;
};
struct WeightedNodalForces {
  std::uint32_t count = 0;
  std::uint32_t nodes[4]{};
  Vec3 forces[4];
  Vec3 couples[4]; // Direct world couples remain identically zero.
};
} // namespace tlfea::contact
