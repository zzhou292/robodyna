// SPDX-License-Identifier: AGPL-3.0-or-later
// OpenRadioss a62b27e6: NORMA4N, NORMV3 and I25NEIGH_REMOVEALLBUT1.
// Original double operation order in native length units; no REAL4 normal reuse.
#pragma once
#include "Internal.h"
#include "../normal_math/DoubleFace.h"
#include <cmath>
namespace tlfea::contact::radioss_type25::startup::detail::neighbor_geometry {
inline constexpr double Em20=1./1.e20;
inline bool Finite(Vector a) noexcept {return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);}
inline bool Normal(const Main& main,const Vector* x,Vector& out) noexcept {
  const Vector points[]{x[main.nodes[0]],x[main.nodes[1]],x[main.nodes[2]],x[main.nodes[3]]};
  normal_math::DoubleFaceResult result;
  if (!normal_math::DoubleFace(points,result)) return false;
  out=result.normal;
  return true;
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
