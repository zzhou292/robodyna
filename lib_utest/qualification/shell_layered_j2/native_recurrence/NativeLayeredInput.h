#pragma once
#include "NativeLayeredReference.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
namespace tl::qualification::layered_native::detail {
using Curve=std::array<double,2050>;
std::mutex& SectionContext() noexcept;
inline bool Disjoint(const void* a,std::size_t an,const void* b,std::size_t bn) noexcept {
  const auto x=reinterpret_cast<std::uintptr_t>(a),y=reinterpret_cast<std::uintptr_t>(b);
  return a&&b&&an<=UINTPTR_MAX-x&&bn<=UINTPTR_MAX-y&&(x+an<=y||y+bn<=x);
}
inline bool Prepare(const Law44& law,const Points& points,Curve& curve) noexcept {
  if(!law.plastic_strain||!law.yield_stress||law.count<2||law.count>1024) return false;
  for(double v:{law.rate_coefficient_per_s,law.rate_exponent,law.cutoff_hz})
    if(!std::isfinite(v)||v<0) return false;
  if(law.rate_coefficient_per_s==0 ? (law.rate_exponent!=0||law.cutoff_hz!=0) :
     (law.rate_exponent<=0)) return false;
  for(std::size_t n=0;n<law.count;++n) {
    const double e=law.plastic_strain[n],y=law.yield_stress[n];
    if(!std::isfinite(e)||!std::isfinite(y)||y<=0||
       (n==0?e!=0:e<=law.plastic_strain[n-1]||y<law.yield_stress[n-1])) return false;
    curve[2*(n+1)]=e;curve[2*(n+1)+1]=y;
  }
  for(const auto& point:points) {
    for(double v:point) if(!std::isfinite(v)) return false;
    if(point[5]<0||point[5]>law.plastic_strain[law.count-1]||point[6]<0||
       (law.rate_coefficient_per_s==0&&point[6]!=0)) return false;
  }
  return true;
}
template<class R,class H,class I,class O>
bool OutputDisjoint(const R& r,const H& h,const I& in,const Law44& law,const O& out) noexcept {
  return law.count<=1024&&Disjoint(&out,sizeof out,&r,sizeof r)&&
    Disjoint(&out,sizeof out,&h,sizeof h)&&Disjoint(&out,sizeof out,&in,sizeof in)&&
    Disjoint(&out,sizeof out,&law,sizeof law)&&
    Disjoint(&out,sizeof out,law.plastic_strain,law.count*sizeof(double))&&
    Disjoint(&out,sizeof out,law.yield_stress,law.count*sizeof(double));
}
inline bool ReadSection(const std::array<double,8>& a,SectionDiagnostics& d) noexcept {
  for(double v:a) if(!std::isfinite(v)) return false;
  if(a[0]<0||a[1]<0||a[2]<a[1]||a[3]<0||a[3]>1||a[4]<0||a[4]>a[3]||
     a[5]<=0||a[6]<=0||a[7]<0) return false;
  d={a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7]};return true;
}
} // namespace tl::qualification::layered_native::detail
