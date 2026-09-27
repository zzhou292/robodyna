// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Reference.h"
#include "../ForceChecks.h"
#include "Material.h"
namespace tl::fea::solid18::total_strain::controlled_distortion {
// Original SI reference and private numeric working-unit geometry are both fixed
// at model startup. Native literal floors therefore execute in native units.
class Reference {
 public:
  TL_SOLID18_HD bool prepared()const noexcept{return prepared_;}
  TL_SOLID18_HD const total_strain::Reference& source()const noexcept{return source_;}
  TL_SOLID18_HD const total_strain::Reference& native_reference()const noexcept{return numeric_;}
  TL_SOLID18_HD const Material& material()const noexcept{return material_;}
 private:
  total_strain::Reference source_,numeric_;
  Material material_;
  bool prepared_=false;
  friend distortion::Status RelocateReference(const Reference&,law::CurveView,Reference&) noexcept;
  friend distortion::Status PrepareReference(const total_strain::Reference&,const Material&,Reference&) noexcept;
};
inline distortion::Status PrepareReference(const total_strain::Reference& source,
    const Material& material,Reference& output) noexcept {
  if(!material.prepared()||!force_detail::ValidMaterial(source,material.source()))
    return distortion::Status::InvalidInput;
  distortion::units_detail::Factors f;
  if(!distortion::units_detail::Make(material.units(),f))return distortion::Status::UnsupportedProfile;
  Reference next;next.source_=source;next.material_=material;
  auto input=source.input();input.density_kg_m3=material.native_material().reader().density_kg_m3;
  for(auto& x:input.position_m)for(unsigned k=0;k<3;++k)
    solid_common::SetComponent(x,k,solid_common::Component(x,k)/f.base.length);
  if(InitializeReference90(input,next.numeric_)!=Status::Success)return distortion::Status::InvalidInput;
  for(unsigned n=0;n<8;++n)if(source.source_slot(n)!=next.numeric_.source_slot(n))
    return distortion::Status::UnsupportedProfile;
  next.prepared_=true;output=next;return distortion::Status::Success;
}
inline distortion::Status RelocateReference(const Reference& source,law::CurveView curve,Reference& output) noexcept {
  if(!source.prepared())return distortion::Status::InvalidInput;
  Reference next=source;
  const auto status=RelocateMaterial(source.material_,curve,next.material_);
  if(status!=distortion::Status::Success)return status;
  output=next;return distortion::Status::Success;
}
} // namespace tl::fea::solid18::total_strain::controlled_distortion
