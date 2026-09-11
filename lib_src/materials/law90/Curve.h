// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "lib_src/materials/law90/Types.h"
#include "lib_src/materials/detail/Vinter2Value.h"

namespace tl::material::law90 {
// Returns the unscaled native VINTER2 value. SIGEPS90 applies YFAC in its own
// material stages; applying scale here would duplicate that operation later.
TL_LAW90_HD inline Status LookupCurve(
    const PreparedMaterial& material, double compression_strain,
    std::uint32_t cursor, CurveResult& output) noexcept {
  if (!material.initialized()) {
    return Status::InvalidInput;
  }
  const CurveView curve = material.curve();
  if (cursor >= curve.count - 1) {
    return Status::InvalidCursor;
  }
  tl::material::detail::Vinter2Result next;
  if (!tl::material::detail::Vinter2Value(
          curve.compression_strain, curve.stress_pa, curve.count,
          compression_strain, cursor, next)) {
    return Status::InvalidCurve;
  }
  output = {next.value, next.slope, next.cursor};
  return Status::Ok;
}
}  // namespace tl::material::law90
