// SPDX-License-Identifier: AGPL-3.0-or-later
// Selected I25NORM/I25NORMP REAL*4 operations. No double normalization or FMA.
#pragma once
#include "Internal.h"
#include "../normal_math/FloatNormals.h"
#include <algorithm>
#include <cmath>
#include <initializer_list>
namespace tlfea::contact::radioss_type25::startup::detail::fp {
using normal_math::Finite;
using normal_math::Add;
using normal_math::Subtract;
using normal_math::Negate;
using normal_math::Cross;
using normal_math::Normalize;
using normal_math::Zero;
inline float StarterFloor() noexcept {
  // The source initializes RDIX=10, REP30=RDIX**30, REM30=1/REP30.
  // Keep this stage distinct from NORMP's literal REAL*4 1.0E-30 floor.
  const float power=std::pow(10.0f,30.0f); return 1.0f/power;
}
using normal_math::ReadyFloor;
inline Report Primary(const Vector* points,Data data,std::size_t p,float floor,
    const PostGapmTopology* post=nullptr) noexcept {
  constexpr unsigned opposite_slot[]{0,3,2,1};
  for(std::size_t m=0;m<p;++m) {
    if(post && post->final_support[m].second_solid_source_id)continue;
    const auto partner=post?data.primary_to_partner[m]:std::uint32_t(p+m+1);
    const auto opposite=partner?std::size_t(partner-1):0;
    const auto& main=data.mains[m];Vector current[4];
    for(unsigned k=0;k<4;++k)current[k]=points[main.nodes[k]];
    const bool quad=main.nodes[2]!=main.nodes[3];StoredNormal normal[4];
    const auto result=normal_math::Primary(current,quad,floor,normal);
    if(!result.valid)return {Status::NonfiniteResult,m,result.bad_corner<4?main.nodes[result.bad_corner]:SIZE_MAX};
    if(quad) {
      for(unsigned k=0;k<4;++k) {
        if(!Finite(normal[k]))return {Status::NonfiniteResult,m};
        data.normals[4*m+k]=normal[k];
        if(partner)data.normals[4*opposite+opposite_slot[k]]=Negate(normal[k]);
      }
    } else {
      if(!Finite(normal[0]))return {Status::NonfiniteResult,m};
      for(unsigned k:{0u,1u,3u}) {
        data.normals[4*m+k]=normal[0];
        if(partner)data.normals[4*opposite+k]=Negate(normal[0]);
      }
      // Original T3 slot3 is initialized positive zero and never negated.
    }
  }
  return {Status::Ok};
}
inline Report FreeEdges(const Vector* points,Data data,std::size_t g,float floor,
    const PostGapmTopology* post=nullptr) noexcept {
  for(std::size_t m=0;m<g;++m) {
    if(post && post->final_support[m].second_solid_source_id)continue;
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      if(main.neighbors[k] || (k==2 && main.nodes[2]==main.nodes[3]))continue;
      const auto a=points[main.nodes[k]],b=points[main.nodes[(k+1)%4]];
      const auto value=normal_math::FreeEdge(a,b,data.normals[4*m+k],floor);
      if(!Finite(value))return {Status::NonfiniteResult,data.expanded_to_primary[m]};
      data.normals[4*m+k]=value;
    }
  }
  return {Status::Ok};
}
inline Report AverageNeighbors(Data data,std::size_t g,float floor,StoredNormal* previous,
    const PostGapmTopology* post=nullptr) noexcept {
  for(std::size_t m=0;m<g;++m) {
    if(post && post->final_support[m].second_solid_source_id)continue;
    const auto& main=data.mains[m];
    for(unsigned k=0;k<4;++k) {
      if(k==2 && main.nodes[2]==main.nodes[3])continue;
      if(main.neighbors[k]) previous[4*m+k]=data.normals[4*std::size_t(main.neighbors[k]-1)+unsigned(main.neighbor_edges[k]-1)];
    }
  }
  for(std::size_t m=0;m<g;++m) {
    if(post && post->final_support[m].second_solid_source_id)continue;
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
