// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid6zForceChecks.h"

namespace tl::fea::solid6z {
TL_BRICK_HD inline Status PreparePrescribedHistory(const Reference& reference,
    const Material& material, const ForceProfile& profile, const HistoryValues& values,
    HistoryStamp stamp, History& output) noexcept {
  if (!reference.prepared()) return Status::InvalidInput;
  if (!force_detail::Valid(profile)) return Status::UnsupportedProfile;
  Material checked;
  if (tl::material::law42::Prepare(material.mu_pa,material.poisson_ratio,
          material.density_kg_m3,material.tension_cutoff_pa,checked) !=
          tl::material::law42::Status::Ok || !force_detail::Same(checked,material) ||
      material.density_kg_m3 != reference.input().density_kg_m3 ||
      !force_detail::Valid(values) || !force_detail::Valid(stamp)) return Status::InvalidInput;
  History next;
  next.data_ = values;
  next.reference_ = reference;
  next.material_ = material;
  next.profile_ = profile;
  next.stamp_ = stamp;
  next.prepared_ = true;
  output = next;
  return Status::Success;
}
TL_BRICK_HD inline Status InitializeHistory(const Reference& reference,
    const Material& material, const ForceProfile& profile, History& output) noexcept {
  HistoryValues values;
  values.material.density_kg_m3 = material.density_kg_m3;
  return PreparePrescribedHistory(reference,material,profile,values,{},output);
}
} // namespace tl::fea::solid6z
