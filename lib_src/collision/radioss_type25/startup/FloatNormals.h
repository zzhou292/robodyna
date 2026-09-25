// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected I25NORM/I25NORMP REAL*4 operations. No double normalization or FMA.
#pragma once
#include "Internal.h"
#include <algorithm>
#include <cmath>
#include <initializer_list>
namespace tlfea::contact::radioss_type25::startup::detail::fp {
inline bool Finite(StoredNormal a) noexcept {
  return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z);
}
inline StoredNormal Add(StoredNormal a,StoredNormal b) noexcept {return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline StoredNormal Subtract(StoredNormal a,StoredNormal b) noexcept {return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline StoredNormal Negate(StoredNormal a) noexcept {return {-a.x,-a.y,-a.z};}
inline StoredNormal Cross(StoredNormal a,StoredNormal b) noexcept {
  return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
inline StoredNormal Normalize(StoredNormal a,float floor) noexcept {
  const float inverse=1.0f/std::max(floor,std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z));
  return {a.x*inverse,a.y*inverse,a.z*inverse};
}
inline bool Zero(StoredNormal a) noexcept {return a.x==0 && a.y==0 && a.z==0;}
inline float StarterFloor() noexcept {
  // The source initializes RDIX=10, REP30=RDIX**30, REM30=1/REP30.
  // Keep this stage distinct from NORMP's literal REAL*4 1.0E-30 floor.
  const float power=std::pow(10.0f,30.0f); return 1.0f/power;
}
inline constexpr float ReadyFloor() noexcept {return 1.0e-30f;}
inline Report Primary(const Vector* points,Data data,std::size_t p,float floor) noexcept {
  constexpr unsigned opposite_slot[]{0,3,2,1};
  for(std::size_t m=0;m<p;++m) {
    const auto& main=data.mains[m];StoredNormal corner[4];
    for(unsigned k=0;k<4;++k) {
      const auto point=points[main.nodes[k]];
      corner[k]={static_cast<float>(point.x),static_cast<float>(point.y),static_cast<float>(point.z)};
      if(!Finite(corner[k]))return {Status::NonfiniteResult,m,main.nodes[k]};
    }
    const bool quad=main.nodes[2]!=main.nodes[3];
    StoredNormal center=corner[2];
    if(quad) center={static_cast<float>(0.25*(corner[0].x+corner[1].x+corner[2].x+corner[3].x)),
        static_cast<float>(0.25*(corner[0].y+corner[1].y+corner[2].y+corner[3].y)),
        static_cast<float>(0.25*(corner[0].z+corner[1].z+corner[2].z+corner[3].z))};
    StoredNormal edge[4],normal[4];
    for(unsigned k=0;k<4;++k)edge[k]=Subtract(corner[k],center);
    for(unsigned k=0;k<4;++k)normal[k]=Normalize(Cross(edge[k],edge[(k+1)%4]),floor);
    if(quad) {
      for(unsigned k=0;k<4;++k) {
        if(!Finite(normal[k]))return {Status::NonfiniteResult,m};
        data.normals[4*m+k]=normal[k];data.normals[4*(p+m)+opposite_slot[k]]=Negate(normal[k]);
      }
    } else {
      if(!Finite(normal[0]))return {Status::NonfiniteResult,m};
      for(unsigned k:{0u,1u,3u}) {
        data.normals[4*m+k]=normal[0];data.normals[4*(p+m)+k]=Negate(normal[0]);
      }
      // Original T3 slot3 is initialized positive zero and never negated.
    }
  }
  return {Status::Ok};
}
inline Report FreeEdges(const Vector* points,Data data,std::size_t g,float floor) noexcept {
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      if(main.neighbors[k] || (k==2 && main.nodes[2]==main.nodes[3]))continue;
      const auto a=points[main.nodes[k]],b=points[main.nodes[(k+1)%4]];
      // Original X is MYREAL8 here: subtract FIRST, then assign to REAL*4.
      const StoredNormal difference{static_cast<float>(b.x-a.x),static_cast<float>(b.y-a.y),static_cast<float>(b.z-a.z)};
      const auto value=Normalize(Cross(difference,data.normals[4*m+k]),floor);
      if(!Finite(value))return {Status::NonfiniteResult,data.expanded_to_primary[m]};
      data.normals[4*m+k]=value;
    }
  }
  return {Status::Ok};
}
inline Report AverageNeighbors(Data data,std::size_t g,float floor,StoredNormal* previous) noexcept {
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      if(k==2 && main.nodes[2]==main.nodes[3])continue;
      if(main.neighbors[k]) previous[4*m+k]=data.normals[4*std::size_t(main.neighbors[k]-1)+unsigned(main.neighbor_edges[k]-1)];
    }
  }
  for(std::size_t m=0;m<g;++m) {
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      if(!main.neighbors[k] || (k==2 && main.nodes[2]==main.nodes[3]))continue;
      const auto value=Normalize(Add(data.normals[4*m+k],previous[4*m+k]),floor);
      if(!Finite(value))return {Status::NonfiniteResult,data.expanded_to_primary[m]};
      data.normals[4*m+k]=value;
    }
  }
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::startup::detail::fp
