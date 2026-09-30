// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid6zGeometry.h"

namespace tl::fea::solid6z {
TL_BRICK_HD inline Status InitializeReference(const ReferenceInput& input, Reference& output) noexcept {
  namespace brick = tl::fea::solid_common;
  const auto& p = input.profile;
  if (p.engine_jhbe != 24 || p.integration_points != 1 || p.strain_formulation != 10 ||
      p.mass_distribution != 0 || p.orthotropic_frame != 0 || p.thermal != 0 ||
      p.ale != 0 || p.reference_shape != 0) return Status::UnsupportedProfile;
  if (!input.source_element_id || !input.source_part_id || !input.source_section_id ||
      !input.source_material_id || !brick::Positive(input.density_kg_m3)) return Status::InvalidInput;
  for (unsigned n = 0; n < 6; ++n) {
    if (!input.source_node_id[n] || !brick::Finite(input.position_m[n])) return Status::InvalidInput;
    for (unsigned j = 0; j < n; ++j) {
      if (input.source_node_id[n] == input.source_node_id[j]) return Status::InvalidInput;
    }
  }
  const auto signed_jacobian = detail::EvaluateJacobian(input.position_m);
  if (!detail::Finite(signed_jacobian)) return Status::NonfiniteResult;
  if (signed_jacobian.volume == 0) return Status::InvalidGeometry;
  Reference next;
  next.input_ = input;
  Vec3 native[6];
  for (unsigned n = 0; n < 6; ++n) {
    const unsigned source = signed_jacobian.volume < 0 ? (n+3)%6 : n;
    next.native_to_source_[n] = static_cast<std::uint8_t>(source);
    native[n] = input.position_m[source];
  }
  // ISMSTR10 retains S6ZCOOR3 world coordinates for S6ZJACIDP, before the
  // unconditional S6ZRCOOR3 local-frame geometry stage.
  const auto reference = detail::EvaluateJacobian(native);
  if (!detail::Finite(reference)) return Status::NonfiniteResult;
  if (!brick::Positive(reference.volume)) return Status::InvalidGeometry;
  if (!detail::ReferenceInverse(reference,next.geometry_)) return Status::NonfiniteResult;
  const Status geometry = detail::LocalGeometry(native,next.geometry_);
  if (geometry != Status::Success) return geometry;

  // S6MASS3 FILL=1, S6FRACA IMAS=0: all three PTG weights are ONE.
  // Its nonzero native slots are 1,2,3,5,6,7; each is one distinct endpoint.
  const double mass = 1.0*input.density_kg_m3*next.geometry_.volume_m3*(1.0/6.0);
  if (!brick::Positive(mass)) return Status::NonfiniteResult;
  for (unsigned n = 0; n < 6; ++n) next.mass_.source_slot_mass_kg[n] = mass*1.0;
  next.mass_.element_mass_kg = 6*mass;
  if (!brick::Positive(next.mass_.element_mass_kg)) return Status::NonfiniteResult;
  next.prepared_ = true;
  output = next;
  return Status::Success;
}
}  // namespace tl::fea::solid6z
