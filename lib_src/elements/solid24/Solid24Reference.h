// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid24Geometry.h"
#include "Solid24ReferenceJacobian.h"
#include "Solid24Topology.h"

namespace tl::fea::solid24 {
TL_BRICK_HD inline Status InitializeReference(const ReferenceInput& input, Reference& output) noexcept {
  namespace brick = tl::fea::solid_common;
  const auto& p = input.profile;
  if (p.engine_jhbe != 24 || p.integration_points != 1 || p.startup_frame != 1 ||
      p.rotational_inertia != 0 || p.ale != 0 || p.reference_shape != 0)
    return Status::UnsupportedProfile;
  if ((p.reference_strain != ReferenceStrain::LocalGeometryOnly &&
       p.reference_strain != ReferenceStrain::TotalLagrangian10) ||
      (p.working_length != WorkingLengthUnit::Metre && p.working_length != WorkingLengthUnit::Millimetre) ||
      (p.reference_strain == ReferenceStrain::LocalGeometryOnly && p.working_length != WorkingLengthUnit::Metre))
    return Status::UnsupportedProfile;
  if (!input.source_element_id || !input.source_part_id || !input.source_section_id ||
      !input.source_material_id || !brick::Positive(input.density_kg_m3)) return Status::InvalidInput;
  unsigned unique_count=0;
  const auto connectivity_status=detail::ValidateConnectivity(input,unique_count);
  if(connectivity_status!=Status::Success)return connectivity_status;
  const double signed_volume = brick::SignedCenterVolume(input.position_m);
  if (!tl::math::Finite(signed_volume)) return Status::NonfiniteResult;
  if (signed_volume == 0) return Status::InvalidGeometry;
  Reference next;
  next.input_ = input;
  next.unique_node_count_ = static_cast<std::uint8_t>(unique_count);
  Vec3 native[8];
  for (unsigned n = 0; n < 8; ++n) {
    const unsigned source = signed_volume < 0 ? (n+4)%8 : n;
    next.native_to_source_[n] = static_cast<std::uint8_t>(source);
    native[n] = input.position_m[source];
  }
  if (p.reference_strain == ReferenceStrain::TotalLagrangian10) {
    const Status status = detail::GlobalReferenceJacobian(native,p.working_length,next.reference_jacobian_);
    if (status != Status::Success) return status;
  }
  if (!detail::CyclicFrame(native,next.geometry_.frame)) return Status::InvalidGeometry;
  for (unsigned n = 0; n < 8; ++n) {
    next.geometry_.local_position_m[n] = brick::Local(next.geometry_.frame,native[n]);
    if (!brick::Finite(next.geometry_.local_position_m[n])) return Status::NonfiniteResult;
  }
  next.geometry_.volume_m3 = detail::CenterVolume(next.geometry_.local_position_m);
  if (!brick::Positive(next.geometry_.volume_m3)) return Status::InvalidGeometry;
  const Status status = detail::CharacteristicLength(next.geometry_);
  if (status != Status::Success) return status;
  // SMASS3 selected FILL=1. All eight native source slots contribute, including aliases.
  const double slot_mass = 1.0*input.density_kg_m3*next.geometry_.volume_m3*(1.0/8.0);
  if (!brick::Positive(slot_mass)) return Status::NonfiniteResult;
  for (unsigned n = 0; n < 8; ++n) next.mass_.source_slot_mass_kg[n] = slot_mass;
  next.mass_.element_mass_kg = 8.0*slot_mass;
  if (!brick::Positive(next.mass_.element_mass_kg)) return Status::NonfiniteResult;
  next.prepared_ = true;
  output = next;
  return Status::Success;
}
}  // namespace tl::fea::solid24
