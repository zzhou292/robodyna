// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Topology.h"
#include "lib_src/elements/solid18/Solid18ReferenceValues.h"

namespace tl::fea::solid18::law44 {
// Same source NIDs retain separate slot contributions. No unique-node mass
// division or six-node conversion is performed by this reference value.
TL_SOLID18_HD inline Status InitializeReference(const ReferenceInput& input,
                                                Reference& output) noexcept {
  if (!detail::Supported(input.profile)) return Status::UnsupportedProfile;
  Reference next;
  if (!detail::ValidateTopology(input, next.topology_)) return Status::InvalidInput;
  next.input_ = input;
  const auto status = solid18::detail::ReferenceGeometryValues(input.position_m,
      input.density_kg_m3, next.geometry_, next.mass_, next.native_to_source_);
  if (status != Status::Success) return status;
  next.prepared_ = true;
  output = next;
  return Status::Success;
}
}  // namespace tl::fea::solid18::law44
