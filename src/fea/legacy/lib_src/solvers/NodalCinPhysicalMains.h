// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/constraints/NodalRigidGroupState.h"
#include <cstddef>
#include <cstdint>

namespace tl::fea {
enum class NodalCinPhysicalMainPolicy : std::uint8_t {
  Unspecified,
  PhysicalAggregateV1
};
// Virtual main values for an actual initialized rigid aggregate, after the
// current CIN coefficient transfer but at pre-kick accepted geometry. No
// physical source node is fabricated or added to the owner domain.
struct NodalCinPhysicalMain {
  RigidBindingSourceKind source_kind = RigidBindingSourceKind::NodalGroup;
  std::uint64_t source_group_id = 0;
  std::uint64_t source_node_set_id = 0;
  tl::math::Vec3 center_m{};
  double mass_kg = 0;
  double minimum_principal_inertia_kg_m2 = 0;
  double translational_stiffness_n_per_m = 0;
  double rotational_stiffness_nm = 0; // Sum(kR + |member-center|^2*kN).
};
struct NodalCinPhysicalMainBuffer {
  NodalCinPhysicalMain* groups = nullptr;
  std::size_t capacity_groups = 0;
};
// Describes a successful query of the actual owner/token; copied values are not
// an independent permission to commit or advance. Only available at TT0 after
// successful AdvanceStaggeredCin, before the first common acceptance.
struct NodalCinPhysicalMainStamp {
  NodalCinPhysicalMainPolicy policy = NodalCinPhysicalMainPolicy::Unspecified;
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  std::uint64_t cin_qualification_id = 0;
  double base_time = 0, owner_fixed_dt = 0;
  NodalRigidGroupInfo groups;
};
} // namespace tl::fea
