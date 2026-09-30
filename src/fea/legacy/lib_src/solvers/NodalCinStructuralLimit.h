// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "NodalCinStructuralStep.h"
#include "../constraints/NodalRigidGroupState.h"

namespace tl::fea {
enum class NodalCinLimitKind : std::uint8_t {
  Unavailable, Unbounded, OrdinaryTranslation, OrdinaryRotation, RigidTrace
};
// Actual post-transfer coefficients at the accepted pre-kick geometry. A rigid
// trace is an upper bound for the scalar stiffness surrogate, not a full
// nonlinear/contact tangent. Its first member is an identity, not a sole cause.
struct NodalCinLimitValues {
  NodalCinLimitKind kind = NodalCinLimitKind::Unavailable;
  std::uint32_t node = UINT32_MAX, group = UINT32_MAX;
  std::uint8_t translation_fixed_bits = 0, rotation_fixed = 0, rotation_present = 0;
  double minimum_dt_s = 0, factor = 0;
  double mass_kg = 0, inertia_kg_m2 = 0;
  double translation_stiffness_n_per_m = 0, rotation_stiffness_nm = 0;
  tl::math::Vec3 principal_inertia_kg_m2{};
  double trace_upper_per_s2 = 0;
};
// A successful, explicitly requested query of one prepared owner attempt.
// Copied diagnostics carry no independent step/commit authority. Public source
// identities are resolved against the owner's retained physical domain/binding.
struct NodalCinStructuralLimit {
  NodalCinLimitValues values;
  std::uint64_t owner_id = 0, base_epoch = 0, attempt = 0;
  std::uint64_t cin_qualification_id = 0, source_instance_id = 0;
  double base_time_s = 0, owner_fixed_dt_s = 0;
  std::uint64_t source_node_id = 0, source_group_id = 0, source_node_set_id = 0;
  RigidBindingSourceKind source_kind = RigidBindingSourceKind::NodalGroup;
};
} // namespace tl::fea
