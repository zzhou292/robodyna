#pragma once
#include "../math/Fixed3.h"
#include <cstddef>
#include <cstdint>

namespace tl::fea {
// Actual world A/AR used by this owner's prepared kick. These values are
// transient force-stage data, not accepted state or reconstructed accelerations.
struct NodalRigidGroupAccelerationSnapshot {
  std::uint64_t source_group_id=0,source_node_set_id=0;
  std::size_t member_count=0;
  tl::math::Vec3 acceleration{},angular_acceleration{};
};
struct NodalForceStageSnapshotBuffer {
  double* acceleration_xyz=nullptr;
  double* angular_acceleration_xyz=nullptr;
  std::size_t capacity_nodes=0;
  NodalRigidGroupAccelerationSnapshot* groups=nullptr;
  std::size_t capacity_groups=0;
};
} // namespace tl::fea
