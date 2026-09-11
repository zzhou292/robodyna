// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Internal.h"
#include "../../../materials/law36/Prepare.h"
#include "../../../materials/law42/Prepare.h"
#include <algorithm>

namespace tl::fea::solids::model_detail {
std::uint64_t MaterialId(ModelInput input,std::size_t index) noexcept {
  if(index<input.solid18.size())return input.solid18[index].reference.input().source_material_id;
  index-=input.solid18.size();
  if(index<input.solid24.size())return input.solid24[index].reference.input().source_material_id;
  return input.solid6z[index-input.solid24.size()].reference.input().source_material_id;
}
ModelReport PlanMaterials(ModelInput input,ModelLimits limits,Scratch& scratch,Layout& layout) {
  const auto count=Count(input);
  scratch.materials.Prepare(count,[&](std::size_t i){return MaterialId(input,i);});
  for(std::size_t i=0;i<count;++i) {
    if(!MaterialId(input,i))return Error(ModelStatus::InvalidInput,"Source MID must be positive",input,i);
    const auto first=scratch.materials.First(MaterialId(input,i));
    if(i<input.solid18.size()) {
      const auto curve=input.solid18[i].material.curve;
      if(curve.count<2 || curve.count>1024 ||
          !Range(curve.plastic_strain,curve.count) || !Range(curve.yield_stress_pa,curve.count))
        return Error(ModelStatus::InvalidInput,"LAW36 curve range is invalid",input,i);
      if(first==i) {
        if(curve.count>limits.max_curve_points-layout.curve_points)
          return Error(ModelStatus::ResourceLimit,"Owned LAW36 curve pool exceeds cap",input,i);
        layout.curve_points+=curve.count;
        scratch.material_indices[i]=layout.count36++;
      } else scratch.material_indices[i]=scratch.material_indices[first];
    } else {
      if(first<input.solid18.size())
        return Error(ModelStatus::MaterialMismatch,"One source MID declares different material laws",input,i);
      scratch.material_indices[i]=first==i?layout.count42++:scratch.material_indices[first];
    }
    if(layout.count36+layout.count42>limits.max_materials)
      return Error(ModelStatus::ResourceLimit,"Unique source material count exceeds cap",input,i);
  }
  return {};
}
ModelReport CopyMaterials(ModelInput input,const Scratch& scratch,const Layout& layout,Storage& out) {
  std::size_t cursor=0;
  for(std::size_t i=0;i<Count(input);++i) {
    const auto mid=MaterialId(input,i);
    const auto first=scratch.materials.First(mid);
    const auto slot=scratch.material_indices[i];
    if(i<input.solid18.size()) {
      const auto& original=input.solid18[i].material;
      auto& record=out.material36[slot];
      if(first==i) {
        record.source_material_id=mid;
        const auto count=original.curve.count;
        auto* x=out.curves+cursor;
        auto* y=x+count;
        cursor+=2*count;
        std::copy_n(original.curve.plastic_strain,count,x);
        std::copy_n(original.curve.yield_stress_pa,count,y);
        if(tl::material::law36::Prepare(original.young_pa,original.poisson_ratio,
            original.density_kg_m3,{x,y,count},record.value)!=tl::material::law36::Status::Ok ||
            !Same(record.value,original))
          return Error(ModelStatus::InvalidInput,"Prepared LAW36 material or curve values differ",input,i);
      } else if(!Same(record.value,original))
        return Error(ModelStatus::MaterialMismatch,"Repeated source MID has different LAW36 values",input,i);
    } else {
      const auto local=i-input.solid18.size();
      const auto& original=local<input.solid24.size()?input.solid24[local].material:
          input.solid6z[local-input.solid24.size()].material;
      auto& record=out.material42[slot];
      if(first==i) {
        record.source_material_id=mid;
        if(tl::material::law42::Prepare(original.mu_pa,original.poisson_ratio,
            original.density_kg_m3,original.tension_cutoff_pa,record.value)!=tl::material::law42::Status::Ok ||
            !Same(record.value,original))
          return Error(ModelStatus::InvalidInput,"Prepared LAW42 material values differ",input,i);
      } else if(!Same(record.value,original))
        return Error(ModelStatus::MaterialMismatch,"Repeated source MID has different LAW42 values",input,i);
    }
  }
  if(cursor!=2*layout.curve_points)
    return Error(ModelStatus::InvalidInput,"Owned curve pool extent differs",input);
  return {};
}
} // namespace tl::fea::solids::model_detail
