// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalRigidGroupMath.h"
#include <cstdint>

namespace tl::fea {
enum class RigidBindingSourceKind { NodalGroup, Part };
struct RigidBindingGroup {
  RigidBindingSourceKind source_kind=RigidBindingSourceKind::NodalGroup;
  std::uint64_t source_id=0,source_node_set_id=0;
  std::size_t member_offset=0,member_count=0;
  double mass_kg=0;
  tl::math::Vec3 center{};
  rigid::PrincipalFrame principal{};
  bool dependent_coefficients=false;
};
struct RigidBindingMember {
  std::uint64_t source_node_id=0;
  std::size_t domain_node=SIZE_MAX;
  tl::math::Vec3 position{};
  double mass_kg=0,isotropic_inertia_kg_m2=0;
};
} // namespace tl::fea
