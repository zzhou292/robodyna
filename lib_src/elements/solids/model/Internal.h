// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "../Model.h"
#include "../../../../lib_utils/BoundedArena.h"
#include "../../../../lib_utils/SourceIdentityIndex.h"
#include <cstring>

namespace tl::fea::solids::model_detail {
using Index = util::SourceIdentityIndex<0>;
struct Layout {
  util::ArenaRegion parent18,parent24,parent6z,material36,material42,curves;
  util::ArenaRegion reference18,reference24,reference6z,material_indices;
  std::size_t arena_bytes=0,scratch_arena_bytes=0,scratch_bytes=0;
  std::size_t fixed_bytes=0,owned_bytes=0,startup_bytes=0;
  std::size_t count36=0,count42=0,curve_points=0;
};
struct Scratch {
  util::HostArena arena;
  Index materials;
  solid18::Reference* reference18=nullptr;
  solid24::Reference* reference24=nullptr;
  solid6z::Reference* reference6z=nullptr;
  std::size_t* material_indices=nullptr;
};
struct Storage {
  util::HostArena arena;
  Parent18* parent18=nullptr;
  Parent24* parent24=nullptr;
  Parent6z* parent6z=nullptr;
  Material36* material36=nullptr;
  Material42* material42=nullptr;
  double* curves=nullptr;
};
inline std::size_t Count(ModelInput input) noexcept {
  return input.solid18.size()+input.solid24.size()+input.solid6z.size();
}
inline ModelReport Error(ModelStatus status,const char* message,ModelInput input,
                         std::size_t index=SIZE_MAX) noexcept {
  if(index==SIZE_MAX)return {status,message,Family::Solid18,index};
  if(index<input.solid18.size())return {status,message,Family::Solid18,index};
  index-=input.solid18.size();
  if(index<input.solid24.size())return {status,message,Family::Solid24,index};
  return {status,message,Family::Solid6z,index-input.solid24.size()};
}
template<class T> bool Range(const T* p,std::size_t count) noexcept {
  if(!count)return p==nullptr;
  const auto address=reinterpret_cast<std::uintptr_t>(p);
  return p && address%alignof(T)==0 && count<=(UINTPTR_MAX-address)/sizeof(T);
}
inline bool Same(double a,double b) noexcept {
  return std::memcmp(&a,&b,sizeof(double))==0;
}
std::uint64_t MaterialId(ModelInput,std::size_t) noexcept;
bool Same(const solid18::Material&,const solid18::Material&) noexcept;
bool Same(const solid24::Material&,const solid24::Material&) noexcept;
bool Same(const solid6z::ForceProfile&,const solid6z::ForceProfile&) noexcept;
ModelReport Preflight(const NodalNodeDomain&,ModelInput,ModelLimits,std::size_t,Layout&);
ModelReport PlanMaterials(ModelInput,ModelLimits,Scratch&,Layout&);
ModelReport CompleteLayout(ModelInput,ModelLimits,Layout&);
ModelReport CopyMaterials(ModelInput,const Scratch&,const Layout&,Storage&);
ModelReport CopyParents(ModelInput,const Scratch&,const SolidNodeContributions&,Storage&);
} // namespace tl::fea::solids::model_detail
