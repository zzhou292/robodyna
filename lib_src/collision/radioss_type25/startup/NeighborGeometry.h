// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6: NORMA4N, NORMV3 and I25NEIGH_REMOVEALLBUT1.
// Original double operation order in native length units; no REAL4 normal reuse.
#pragma once
#include "Internal.h"
#include <cmath>
namespace tlfea::contact::radioss_type25::startup::detail::neighbor_geometry {
inline constexpr double Em20=1./1.e20;
inline bool Finite(Vector a) noexcept {return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
inline bool Normal(const Main& main,const Vector* x,Vector& out) noexcept {
  const auto a=x[main.nodes[0]],b=x[main.nodes[1]],c=x[main.nodes[2]],d=x[main.nodes[3]];
  const Vector u{c.x-a.x,c.y-a.y,c.z-a.z},v{d.x-b.x,d.y-b.y,d.z-b.z};
  const Vector n{u.y*v.z-u.z*v.y,u.z*v.x-u.x*v.z,u.x*v.y-u.y*v.x};
  const double raw=std::sqrt(n.x*n.x+n.y*n.y+n.z*n.z);
  if(!Finite(u)||!Finite(v)||!Finite(n)||!std::isfinite(raw))return false;
  const double area=raw>Em20?raw:Em20;
  out={n.x/area,n.y/area,n.z/area};return Finite(out);
}
inline bool Normalize(Vector& v) noexcept {
  const double norm=std::sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
  if(!Finite(v)||!std::isfinite(norm))return false;
  const double inverse=1./(norm>Em20?norm:Em20);
  v={v.x*inverse,v.y*inverse,v.z*inverse};return Finite(v);
}
inline bool Scores(const Main& self,const Main* mains,const Vector* x,
    std::uint32_t i1,std::uint32_t i2,const int* ids,std::size_t count,
    double* angles,double* sides) noexcept {
  Vector ni;if(!Normal(self,x,ni))return false;
  const auto a=x[i1],b=x[i2];const Vector delta{b.x-a.x,b.y-a.y,b.z-a.z};
  Vector yi{ni.y*delta.z-ni.z*delta.y,ni.z*delta.x-ni.x*delta.z,ni.x*delta.y-ni.y*delta.x};
  if(!Finite(delta)||!Normalize(yi))return false;
  for(std::size_t i=0;i<count;++i) {
    Vector nj;if(!Normal(mains[ids[i]-1],x,nj))return false;
    Vector yj{-nj.y*delta.z+nj.z*delta.y,-nj.z*delta.x+nj.x*delta.z,-nj.x*delta.y+nj.y*delta.x};
    if(!Normalize(yj))return false;
    sides[i]=ni.x*yj.x+ni.y*yj.y+ni.z*yj.z;
    angles[i]=yi.x*yj.x+yi.y*yj.y+yi.z*yj.z;
    if(!std::isfinite(sides[i])||!std::isfinite(angles[i]))return false;
  }
  return true;
}
// Return the original first winning ordinal, or SIZE_MAX (native IRR11).
inline std::size_t Winner(const double* angle,const double* side,std::size_t count) noexcept {
  std::size_t positive=0,winner=SIZE_MAX;
  for(std::size_t i=0;i<count;++i)if(side[i]>=0)++positive;
  double maximum=-(1.+1./100.);
  for(std::size_t i=0;i<count;++i) {
    if(side[i]<0)continue;
    if(angle[i]>=-1. && maximum<angle[i]){maximum=angle[i];winner=i;}
  }
  if(winner==SIZE_MAX && positive>0) {
    double minimum=1.e10;
    for(std::size_t i=0;i<count;++i) {
      if(side[i]<0)continue;
      if(minimum>angle[i]){minimum=angle[i];winner=i;}
    }
  }
  if(positive!=count && (positive==0 || winner==SIZE_MAX)) {
    maximum=-(1.+1./100.);
    for(std::size_t i=0;i<count;++i) {
      if(side[i]>=0)continue;
      if(angle[i]<-1. && maximum<angle[i]){maximum=angle[i];winner=i;}
    }
    if(winner==SIZE_MAX) {
      double minimum=1.e10;
      for(std::size_t i=0;i<count;++i) {
        if(side[i]>=0)continue;
        if(angle[i]>=-1. && minimum>angle[i]){minimum=angle[i];winner=i;}
      }
    }
  }
  return winner;
}
} // namespace tlfea::contact::radioss_type25::startup::detail::neighbor_geometry
