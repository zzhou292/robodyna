// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid18ReferenceValues.h"

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
  Reference next;
  next.input_ = input;
  const Status status = detail::ReferenceValues(input, next.geometry_, next.mass_,
                                               next.native_to_source_);
  if (status != Status::Success) return status;
  next.prepared_ = true;
  output = next;
  return Status::Success;
}
}  // namespace tl::fea::solid18
