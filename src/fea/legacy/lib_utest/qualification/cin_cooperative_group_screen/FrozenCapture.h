// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/solvers/cin_limiter/Witness.h"
#include "FrozenValues.h"

namespace tl::fea::cooperative_test::frozen_limiter {
using cin_limiter::Witness;
// Observational only: called after the unchanged screen. A diagnostic failure
// leaves an unavailable witness and never changes the screen's status or dt.
TL_SURFACE_HD inline Witness Capture(const cin_timestep::Sources& source,
    double factor, const cin_timestep::Result& result,
    std::uint64_t epoch, std::uint64_t attempt) noexcept {
  Witness out;
  if (!result.valid || !attempt) return out;
  auto& value = out.values;
  value.minimum_dt_s = result.minimum_dt;
  value.factor = factor;
  value.node = result.limiting_node;
  value.group = result.limiting_group;
  if (value.node == UINT32_MAX && value.group == UINT32_MAX &&
      value.minimum_dt_s == std::numeric_limits<double>::max()) {
    value.kind = NodalCinLimitKind::Unbounded;
  } else if (value.node >= source.nodes) {
    return {};
  } else if (value.group != UINT32_MAX) {
    if (value.group >= source.rigid.group_count) return {};
    double trace = 0;
    const auto group = frozen::EvaluateGroup(source, factor, value.group, &trace);
    if (!group.valid || !group.limit.bounded || group.first_node != value.node ||
        group.limit.dt != value.minimum_dt_s) return {};
    const auto range = source.rigid.groups[value.group];
    value.kind = NodalCinLimitKind::RigidTrace;
    value.mass_kg = range.mass;
    value.principal_inertia_kg_m2 = range.principal_inertia;
    value.trace_upper_per_s2 = trace;
  } else {
    cin_timestep::Result checked;
    if (!frozen::EvaluateNode(source, factor, value.node, checked) ||
        checked.minimum_dt != value.minimum_dt_s || checked.limiting_node != value.node)
      return {};
    value.mass_kg = source.mass[value.node];
    value.translation_fixed_bits = source.fixed_translation[value.node];
    value.rotation_fixed = source.fixed_rotation[value.node];
    value.rotation_present = source.rotation_present ? source.rotation_present[value.node] : 1;
    value.inertia_kg_m2 = source.inertia[value.node];
    value.translation_stiffness_n_per_m = source.translation[value.node];
    value.rotation_stiffness_nm = source.rotation[value.node];
    cin_timestep::ScalarLimit translation;
    const bool wins_translation = source.fixed_translation[value.node] != 7 &&
        cin_timestep::OrdinaryLimit(value.mass_kg, value.translation_stiffness_n_per_m,
            factor, translation) && translation.bounded && translation.dt == value.minimum_dt_s;
    // The original screen checks translation first and only replaces on <.
    value.kind = wins_translation ? NodalCinLimitKind::OrdinaryTranslation
                                 : NodalCinLimitKind::OrdinaryRotation;
  }
  out.epoch = epoch;
  out.attempt = attempt;
  return out;
}
} // namespace tl::fea::cin_limiter
