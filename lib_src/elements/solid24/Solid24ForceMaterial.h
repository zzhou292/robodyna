// SPDX-License-Identifier: AGPL-3.0-or-later
// Shared selected HEPH material caller binding; no material arithmetic here.
#pragma once
#include "Solid24ForceTypes.h"

namespace tl::fea::solid24::force_detail {
TL_BRICK_HD inline ForceStatus EvaluateMaterial(const Reference& reference,
    const Material& material,const ForceGeometry& geometry,double dt_s,
    const tl::material::law42::CallerHistory& accepted_material,bool initialization,
    tl::material::law42::CallerResult& result) noexcept {
  tl::material::law42::CallerInput input;
  input.dt_s=dt_s;
  input.current_volume_m3=geometry.current.volume_m3;
  input.storage_volume_m3=reference.geometry().volume_m3;
  input.characteristic_length_m=geometry.current.characteristic_length_m;
  for (unsigned k=0; k<9; ++k)
    input.displacement_gradient[k]=geometry.material_displacement_gradient[k];
  for (unsigned k=0; k<6; ++k)
    input.engineering_rate_per_s[k]=geometry.engineering_rate_per_s[k];
  const auto material_status=initialization ?
      tl::material::law42::InitializeCaller(material,input,result) :
      tl::material::law42::UpdateCaller(material,accepted_material,input,result);
  if (material_status!=tl::material::law42::Status::Ok) return ForceStatus::MaterialFailure;
  return ForceStatus::Success;
}
} // namespace tl::fea::solid24::force_detail
