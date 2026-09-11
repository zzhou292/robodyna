// SPDX-License-Identifier: AGPL-3.0-or-later
// Shared native startup geometry/M; public wrappers own exact profile admission.
#pragma once
#include "Solid18StartupGeometry.h"
#include "Solid18Mass.h"

namespace tl::fea::solid18::detail {
// Requires validated finite source-slot positions and positive density. Topology
// and formulation admission remain the caller's responsibility.
TL_SOLID18_HD inline Status ReferenceGeometryValues(const Vec3 (&position_m)[8],
    double density_kg_m3, StartupGeometry& geometry, Mass& mass,
    std::uint8_t (&native_to_source)[8]) noexcept {
  const double center_volume = SignedCenterVolume(position_m);
  if (!tl::math::Finite(center_volume)) return Status::NonfiniteResult;
  if (center_volume == 0) return Status::InvalidGeometry;
  Vec3 native[8];
  for (unsigned n = 0; n < 8; ++n) {
    const unsigned source = center_volume < 0 ? (n+4)%8 : n;
    native_to_source[n] = static_cast<std::uint8_t>(source);
    native[n] = position_m[source];
  }
  if (!Frame(native, geometry.frame)) return Status::InvalidGeometry;
  for (unsigned n = 0; n < 8; ++n) {
    geometry.native_position_m[n] = Local(geometry.frame, native[n]);
    if (!Finite(geometry.native_position_m[n])) return Status::NonfiniteResult;
  }
  Status status = IntegrateGeometry(geometry);
  if (status != Status::Success) return status;
  return PrepareMass(density_kg_m3, geometry, native_to_source, mass);
}

TL_SOLID18_HD inline Status ReferenceValues(const ReferenceInput& input,
    StartupGeometry& geometry, Mass& mass,
    std::uint8_t (&native_to_source)[8]) noexcept {
  if (!input.source_element_id || !input.source_part_id || !input.source_section_id ||
      !input.source_material_id || !Positive(input.density_kg_m3))
    return Status::InvalidInput;
  for (unsigned n = 0; n < 8; ++n) {
    if (!input.source_node_id[n] || !Finite(input.position_m[n]))
      return Status::InvalidInput;
    for (unsigned prior = 0; prior < n; ++prior) {
      if (input.source_node_id[prior] == input.source_node_id[n]) return Status::InvalidInput;
    }
  }
  return ReferenceGeometryValues(input.position_m, input.density_kg_m3,
                                 geometry, mass, native_to_source);
}
}  // namespace tl::fea::solid18::detail
