// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ReferencePij.h"
#include "lib_src/elements/solid18/Solid18ReferenceValues.h"

namespace tl::fea::solid18::total_strain {
// External per-operation storage. Inputs and accepted destinations must be disjoint
// from this scratch. Failed operations may modify scratch, never accepted state.
struct ReferenceScratch {
  Reference staged;
  StartupGeometry world;
  CurrentGeometry derivatives;
};

namespace detail {
TL_SOLID18_HD inline bool Supported(const ResolvedProfile& p) noexcept {
  return p.material_law == 90 && p.native_isolid == 18 && p.engine_jhbe == 17 &&
      p.integration == 2 && p.nptr == 2 && p.npts == 2 && p.nptt == 2 &&
      p.pressure == 0 && p.small_strain == 10 && p.convected_frame == 1;
}
}

// Pure selected startup; no material histories or source/owner admission.
TL_SOLID18_HD inline Status InitializeReference90Scratch(const ReferenceInput& input,
                                                        ReferenceScratch& scratch) noexcept {
  if (!detail::Supported(input.profile)) return Status::UnsupportedProfile;
  auto& next = scratch.staged;
  next.prepared_ = false;
  next.input_ = input;
  auto status = solid18::detail::ReferenceValues(input, next.geometry_, next.mass_,
                                                next.native_to_source_);
  if (status != Status::Success) return status;
  // INITIA's SGSAVINI precedes the later H8 orientation correction.
  for (unsigned n = 0; n < 7; ++n) {
    const auto& x = input.position_m[n];
    const auto& last = input.position_m[7];
    next.coefficients_.source_relative_position_m[n] = {x.x-last.x, x.y-last.y, x.z-last.z};
    if (!solid18::detail::Finite(next.coefficients_.source_relative_position_m[n]))
      return Status::NonfiniteResult;
  }
  auto& world = scratch.world;
  for (unsigned n = 0; n < 8; ++n)
    world.native_position_m[n] = input.position_m[next.native_to_source_[n]];
  status = solid18::detail::CenterGeometry(world);
  if (status != Status::Success) return status;
  Matrix3 inverse;
  solid18::detail::InverseScaledJacobian(world.center_scaled_jacobian_m,
      world.center_volume_m3, 1./64., inverse);
  for (unsigned k = 0; k < 9; ++k) {
    if (!tl::math::Finite(inverse.v[k])) return Status::NonfiniteResult;
    next.coefficients_.center_jacobian_inverse_per_m[k] = inverse.v[k];
  }
  next.coefficients_.center_determinant_m3 = world.center_volume_m3;
  status = detail::ReferencePointCoefficients(next.geometry_, next.coefficients_, scratch.derivatives);
  if (status != Status::Success) return status;
  next.prepared_ = true;
  return Status::Success;
}

// Host convenience publishes only a fully prepared value. Device callers own
// ReferenceScratch explicitly and publish scratch.staged after success.
inline Status InitializeReference90(const ReferenceInput& input, Reference& output) noexcept {
  ReferenceScratch scratch;
  const auto status = InitializeReference90Scratch(input, scratch);
  if (status == Status::Success) output = scratch.staged;
  return status;
}
}  // namespace tl::fea::solid18::total_strain
