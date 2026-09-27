// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Solid6zForceTypes.h"
#include "lib_src/materials/law42/Caller.h"
namespace tl::fea::solid6z::force_detail {
TL_BRICK_HD inline Status EvaluateMaterial(const Reference& reference,const Material& material,
    const CurrentGeometry& geometry,double dt_s,const tl::material::law42::CallerHistory& accepted_material,
    bool initialization,tl::material::law42::CallerResult& result)noexcept {
  tl::material::law42::CallerInput input;
  for (unsigned k = 0; k < 9; ++k) input.displacement_gradient[k] = geometry.material_displacement_gradient[k];
  for (unsigned k = 0; k < 6; ++k) input.engineering_rate_per_s[k] = geometry.engineering_rate_per_s[k];
  input.dt_s = dt_s;
  input.current_volume_m3 = geometry.current_volume_m3;
  input.storage_volume_m3 = reference.geometry().volume_m3;
  input.characteristic_length_m = geometry.characteristic_length_m;
  const auto material_status=initialization ?
      tl::material::law42::InitializeCaller(material,input,result) :
      tl::material::law42::UpdateCaller(material,accepted_material,input,result);
  if (material_status!=tl::material::law42::Status::Ok) return Status::NonfiniteResult;
  return Status::Success;
}
} // namespace tl::fea::solid6z::force_detail
