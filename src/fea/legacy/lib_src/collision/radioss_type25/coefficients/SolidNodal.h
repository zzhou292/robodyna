// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6 SBULK3 NNC6/8; see ../LICENSE.md.
#pragma once
#include "Common.h"
#include "../NodalContributionTypes.h"

namespace tlfea::contact::radioss_type25 {
TL_MATH_HOST_DEVICE inline CoefficientStatus EvaluateNativeSolidNodalShares(
    const NativeSolidNodalInput& in, NativeSolidNodalShares* output) {
  using namespace coefficient_detail;
  if (!output) return CoefficientStatus::InvalidInput;
  if (in.kind != SolidNodalKind::Hex8 && in.kind != SolidNodalKind::Penta6)
    return CoefficientStatus::UnsupportedProfile;
  if (!Finite(in.volume) || !Finite(in.fill) || !Finite(in.bulk))
    return CoefficientStatus::InvalidInput;

  NativeSolidNodalShares result;
  result.volume_share = in.volume * (in.kind == SolidNodalKind::Hex8 ? 1. / 8. : 1. / 6.);
  const double filled_bulk = in.fill * in.bulk;
  // Do not let overflow followed by zero hide an invalid arithmetic result.
  if (!Finite(result.volume_share) || !Finite(filled_bulk))
    return CoefficientStatus::NonfiniteResult;
  result.bulk_volume_share = filled_bulk * result.volume_share;
  if (!Finite(result.bulk_volume_share)) return CoefficientStatus::NonfiniteResult;
  result.defined_raw_slot_mask = in.kind == SolidNodalKind::Hex8 ? std::uint8_t{0xff} : std::uint8_t{0x77};
  *output = result;
  return CoefficientStatus::Ok;
}
} // namespace tlfea::contact::radioss_type25
