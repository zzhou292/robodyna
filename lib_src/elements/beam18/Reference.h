// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Geometry.h"
#include "Mass.h"

namespace tl::fea::beam18 {
TL_BEAM18_HD inline Status InitializeReference(const Input& input, Reference& output) noexcept {
  detail::UnitFactors units;
  if (!detail::Units(input.units,units) || !input.source_element_id || !input.source_part_id ||
      !input.source_section_id || !input.source_material_id || !input.source_node_id[0] ||
      !input.source_node_id[1] || input.source_node_id[0] == input.source_node_id[1] ||
      !detail::Positive(input.radius) || !detail::Positive(input.density) || !detail::Positive(input.young) ||
      !tl::math::Finite(input.poisson) || input.poisson <= -1 || input.poisson >= .5)
    return Status::InvalidInput;
  if (input.profile != Profile::CircularFourPointStoredZero || input.local != 2)
    return Status::UnsupportedScope;
  for (unsigned release : input.release) if (release) return Status::UnsupportedScope;
  for (const auto position : input.position)
    if (!tl::math::fixed3::Finite(position)) return Status::InvalidInput;
  for (unsigned i = 0; i < 2; ++i)
    if (input.source_node_id[2] == input.source_node_id[i] &&
        !detail::SamePosition(input.position[2],input.position[i])) return Status::InvalidInput;
  if (!input.source_node_id[2] && !detail::SamePosition(input.position[2],{})) return Status::InvalidInput;
  Reference next{};
  next.input_ = input;
  auto status = detail::PrepareSection(input.radius,next.section_);
  if (status != Status::Success) return status;
  status = detail::PrepareGeometry(input,units,next.geometry_);
  if (status != Status::Success) return status;
  status = detail::PrepareMass(input,next.section_,next.geometry_,units,next.native_mass_,next.endpoint_);
  if (status != Status::Success) return status;
  next.prepared_ = true;
  output = next;
  return Status::Success;
}
} // namespace tl::fea::beam18
