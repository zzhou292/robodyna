// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "Families.h"
#include "../../../../lib_utils/BoundedArena.h"
#include "../../../../lib_utils/SourceIdentityIndex.h"
#include <cstring>

namespace tl::fea::solids::model_detail {
using Index = util::SourceIdentityIndex<0>;
struct Layout {
  control::Budget control_budget;
  util::ArenaRegion parent18,parent24,parent6z,material36,material42,curves;
  util::ArenaRegion parent44,parent90,material44,material90;
  util::ArenaRegion reference18,reference24,reference6z,reference44,reference90,material_indices;
  std::size_t arena_bytes=0,scratch_arena_bytes=0,scratch_bytes=0;
  std::size_t fixed_bytes=0,owned_bytes=0,startup_bytes=0;
  std::size_t count36=0,count42=0,count44=0,count90=0,curve_points=0;
};
struct Scratch {
  util::HostArena arena;
  Index materials;
  solid18::Reference* reference18=nullptr;
  solid24::Reference* reference24=nullptr;
  solid6z::Reference* reference6z=nullptr;
  solid18::law44::Reference* reference44=nullptr;
  solid18::total_strain::Reference* reference90=nullptr;
  std::size_t* material_indices=nullptr;
};
struct Storage {
  util::HostArena arena;
  Parent18* parent18=nullptr;
  Parent24* parent24=nullptr;
  Parent6z* parent6z=nullptr;
  Parent18Law44* parent44=nullptr;
  Parent18Law90* parent90=nullptr;
  Material44* material44=nullptr;
  Material90* material90=nullptr;
  Material36* material36=nullptr;
  Material42* material42=nullptr;
  double* curves=nullptr;
};
inline ModelReport Error(ModelStatus status,const char* message,ModelInput input,
                         std::size_t index=SIZE_MAX) noexcept {
  if(index==SIZE_MAX)return {status,message,Family::Solid18,index};
  const auto at = Locate(input,index);
  return {status,message,at.family,at.local};
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
struct CurveSpan { const double* x=nullptr; const double* y=nullptr; std::size_t count=0; };
inline CurveSpan Curve(const solid18::Material& m) { return {m.curve.plastic_strain,m.curve.yield_stress_pa,m.curve.count}; }
inline CurveSpan Curve(const solid24::Material&) { return {}; }
inline CurveSpan Curve(const solid18::law44::Material& m) { return {m.curve.plastic_strain,m.curve.yield_stress_pa,m.curve.count}; }
template<class Material> inline bool RequiresCurve(const Material&) { return true; }
inline bool RequiresCurve(const solid24::Material&) { return false; }
inline bool RequiresCurve(const solid18::law44::Material& m) {
  return m.material.hardening != tl::material::law44::solid::HardeningKind::Analytic;
}
inline CurveSpan Curve(const tl::material::law90::PreparedMaterial& m) {
  const auto c=m.curve(); return {c.compression_strain,c.stress_pa,c.count};
}
inline std::size_t& MaterialCount(Layout& l,const solid18::Material&) { return l.count36; }
inline std::size_t& MaterialCount(Layout& l,const solid24::Material&) { return l.count42; }
inline std::size_t& MaterialCount(Layout& l,const solid18::law44::Material&) { return l.count44; }
inline std::size_t& MaterialCount(Layout& l,const tl::material::law90::PreparedMaterial&) { return l.count90; }
CurveSpan CopyCurve(CurveSpan,std::size_t&,Storage&);
bool Same(const solid18::law44::Material&,const solid18::law44::Material&) noexcept;
bool Same(const tl::material::law90::PreparedMaterial&,const tl::material::law90::PreparedMaterial&) noexcept;
bool SameCurves(CurveSpan,CurveSpan) noexcept;
ModelReport CopyMaterial(const solid18::Material&,std::uint64_t,bool,std::size_t,std::size_t&,Storage&);
ModelReport CopyMaterial(const solid24::Material&,std::uint64_t,bool,std::size_t,std::size_t&,Storage&);
ModelReport CopyMaterial(const solid18::law44::Material&,std::uint64_t,bool,std::size_t,std::size_t&,Storage&);
ModelReport CopyMaterial(const tl::material::law90::PreparedMaterial&,std::uint64_t,bool,std::size_t,std::size_t&,Storage&);
ModelReport Preflight(const NodalNodeDomain&,ModelInput,ModelLimits,std::size_t,Layout&);
ModelReport PlanMaterials(ModelInput,ModelLimits,Scratch&,Layout&);
ModelReport CompleteLayout(ModelInput,ModelLimits,Layout&);
ModelReport CopyMaterials(ModelInput,const Scratch&,const Layout&,Storage&);
ModelReport CopyParents(ModelInput,const Scratch&,const SolidNodeContributions&,Storage&);
} // namespace tl::fea::solids::model_detail
