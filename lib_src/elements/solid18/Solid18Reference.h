// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18StartupGeometry.h"
#include "Solid18Mass.h"

namespace tl::fea::solid18 {
namespace detail {
TL_SOLID18_HD inline bool Supported(const ResolvedProfile& p) noexcept {
  return p.material_law == 36 && p.native_isolid == 18 && p.engine_jhbe == 17 &&
         p.integration == 2 && p.nptr == 2 && p.npts == 2 && p.nptt == 2 &&
         p.pressure == 2 && p.small_strain == 2 && p.convected_frame == 1;
}
}  // namespace detail

// Pure native startup. Even an input alias into a previous Reference is read
// completely before the final publication; failures leave output untouched.
TL_SOLID18_HD inline Status InitializeReference(const ReferenceInput& input,
                                                Reference& output) noexcept {
  if (!detail::Supported(input.profile)) return Status::UnsupportedProfile;
  if (!input.source_element_id || !input.source_part_id || !input.source_section_id ||
      !input.source_material_id || !detail::Positive(input.density_kg_m3))
    return Status::InvalidInput;
  for (unsigned n = 0; n < 8; ++n) {
    if (!input.source_node_id[n] || !detail::Finite(input.position_m[n]))
      return Status::InvalidInput;
    for (unsigned prior = 0; prior < n; ++prior) {
      if (input.source_node_id[prior] == input.source_node_id[n]) return Status::InvalidInput;
    }
  }
  Reference next;
  next.input_ = input;
  const double center_volume = detail::SignedCenterVolume(input.position_m);
  if (!tl::math::Finite(center_volume)) return Status::NonfiniteResult;
  if (center_volume == 0) return Status::InvalidGeometry;
  Vec3 native[8];
  for (unsigned n = 0; n < 8; ++n) {
    const unsigned source = center_volume < 0 ? (n+4)%8 : n;
    next.native_to_source_[n] = static_cast<std::uint8_t>(source);
    native[n] = input.position_m[source];
  }
  if (!detail::Frame(native, next.geometry_.frame)) return Status::InvalidGeometry;
  for (unsigned n = 0; n < 8; ++n) {
    next.geometry_.native_position_m[n] = detail::Local(next.geometry_.frame, native[n]);
    if (!detail::Finite(next.geometry_.native_position_m[n])) return Status::NonfiniteResult;
  }
  Status status = detail::IntegrateGeometry(next.geometry_);
  if (status != Status::Success) return status;
  status = detail::PrepareMass(input.density_kg_m3, next.geometry_, next.native_to_source_, next.mass_);
  if (status != Status::Success) return status;
  next.prepared_ = true;
  output = next;
  return Status::Success;
}
}  // namespace tl::fea::solid18
