// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../solid18/Solid18ForceTypes.h"
#include "../solid24/Solid24ForceTypes.h"
#include "../solid6z/Solid6zForceTypes.h"

namespace tl::fea::solids {
struct NodalStiffness {
  double translation_n_m = 0;
  double rotation_nm = 0;
};
namespace stiffness_detail {
TL_BRICK_HD inline bool Prepare(double raw, double dt, double factor,
                                NodalStiffness& output) noexcept {
  if (!tl::math::Finite(raw) || raw <= 0 || !tl::math::Finite(dt) || dt <= 0)
    return false;
  const double value=factor*raw;
  if (!tl::math::Finite(value) || value <= 0) return false;
  output={value,0};
  return true;
}
} // namespace stiffness_detail
// Native SCUMU3 scales the complete eight-point STI sum once. Do not average
// point timesteps or reconstruct STI from the aggregate nodal mass.
TL_BRICK_HD inline bool PrepareNodalStiffness(const solid18::ForceTrial& value,
                                             NodalStiffness& output) noexcept {
  return value.proposed_history.prepared() && stiffness_detail::Prepare(
      value.diagnostics.raw_stiffness_n_m,value.diagnostics.minimum_unscaled_dt_s,.25,output);
}
TL_BRICK_HD inline bool PrepareNodalStiffness(const solid24::ForceTrial& value,
                                             NodalStiffness& output) noexcept {
  return value.proposed_history.initialized() && stiffness_detail::Prepare(
      value.diagnostics.material.raw_stiffness_n_m,
      value.diagnostics.material.unscaled_element_dt_s,.25,output);
}
// S6CUMU3 consumes six actual slots; its THIRD is a binary64 native constant.
TL_BRICK_HD inline bool PrepareNodalStiffness(const solid6z::ForceTrial& value,
                                             NodalStiffness& output) noexcept {
  return value.proposed_history.prepared() && stiffness_detail::Prepare(
      value.material.raw_stiffness_n_m,value.material.unscaled_element_dt_s,1.0/3.0,output);
}
} // namespace tl::fea::solids
