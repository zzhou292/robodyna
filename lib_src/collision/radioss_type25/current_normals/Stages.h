// SPDX-License-Identifier: AGPL-3.0-or-later
// Derived from OpenRadioss, Copyright (C) 2026 Siemens; see ../LICENSE.md.
#pragma once
#include "Types.h"
#include "../normal_math/FloatNormals.h"
namespace tlfea::contact::radioss_type25::current_normals::detail {
namespace fp=normal_math;
struct Work {
  StoredNormal* normal=nullptr;
  StoredNormal* neighbor=nullptr;
  unsigned char* eligible=nullptr;
  unsigned char* tage=nullptr;
  std::uint32_t* slots=nullptr; // Two original edge indices per reference.
  startup::NormalReference* references=nullptr;
};
TL_MATH_HOST_DEVICE inline Vector Position(const Input& in,std::uint32_t node,double length) {
  const auto x=in.positions.at(node);
  if(in.coordinates==startup::Coordinates::Native)return {x.x,x.y,x.z};
  return {x.x/length,x.y/length,x.z/length};
}
TL_MATH_HOST_DEVICE inline bool FreeEdge(const startup::Main& m,unsigned edge) {
  return m.neighbors[edge]==0&&!(edge==2&&m.nodes[2]==m.nodes[3]);
}
TL_MATH_HOST_DEVICE inline bool FreeMain(const Input& in,std::size_t main) {
  if(in.main_coefficients[main]<=0)return false;
  for(unsigned edge=0;edge<4;++edge)if(FreeEdge(in.topology.mains[main],edge))return true;
  return false;
}
// Each primary and its authenticated unique opposite have one writer. A skipped
// primary preserves every accepted bit, including the unused T3 slot.
TL_MATH_HOST_DEVICE inline Report Primary(const Input& in,Work w,std::size_t main,double length) {
  const auto& m=in.topology.mains[main];bool tagged=false;
  for(auto node:m.nodes)tagged=tagged||in.node_tag[node]!=0;
  w.tage[main]=tagged?0:1;if(!tagged)return {Status::Ok};
  StoredNormal normal[4]{};const bool quad=m.nodes[2]!=m.nodes[3];
  if(in.main_coefficients[main]>0) {
    Vector x[4];for(unsigned k=0;k<4;++k)x[k]=Position(in,m.nodes[k],length);
    const auto result=fp::Primary(x,quad,fp::ReadyFloor(),normal);
    if(!result.valid)return {Status::NonfiniteResult,main,SIZE_MAX,result.bad_corner<4?m.nodes[result.bad_corner]:SIZE_MAX};
  }
  const auto opposite=std::size_t(m.segment_type-1);
  constexpr unsigned reverse[]{0,3,2,1};
  for(unsigned k=0;k<4;++k) {
    if(!quad&&k==2)continue;
    const auto value=quad?normal[k]:normal[0];
    w.normal[4*main+k]=value;
    w.normal[4*opposite+(quad?reverse[k]:k)]=fp::Negate(value);
  }
  return {Status::Ok};
}
// Retain raw eligibility BEFORE converting free-face normals into edge vectors.
TL_MATH_HOST_DEVICE inline void FreeEligibility(const Input& in,Work w,std::size_t item) {
  const auto main=std::size_t(in.free_main_ids[item]-1);
  for(unsigned k=0;k<4;++k)
    if(FreeEdge(in.topology.mains[main],k))w.eligible[4*main+k]=fp::Zero(w.normal[4*main+k])?0:1;
}
// The exact ordered CSR contains each main/reference only once. Thus each
// reference sees precisely its subsequence of the native ascending main/edge
// traversal. Admission also proves distinct node-bound endpoint references.
TL_MATH_HOST_DEVICE inline void ReferenceSlots(const Input& in,Work w,std::size_t ref) {
  auto& output=w.references[ref];output={};
  const auto& csr=in.topology.normal_to_main;
  for(auto i=csr.offsets[ref];i<csr.offsets[ref+1];++i) {
    const auto main=std::size_t(csr.entries[i]-1);const auto& m=in.topology.mains[main];
    if(in.main_coefficients[main]<=0)continue;
    for(unsigned k=0;k<4;++k) {
      if(!FreeEdge(m,k)||!w.eligible[4*main+k])continue;
      if(std::size_t(m.normal_reference[k]-1)!=ref&&std::size_t(m.normal_reference[(k+1)%4]-1)!=ref)continue;
      ++output.boundary;
      if(output.boundary<=2)w.slots[2*ref+std::size_t(output.boundary-1)]=std::uint32_t(4*main+k);
    }
  }
}
TL_MATH_HOST_DEVICE inline Report TransformFree(const Input& in,Work w,std::size_t item,double length) {
  const auto main=std::size_t(in.free_main_ids[item]-1);const auto& m=in.topology.mains[main];
  for(unsigned k=0;k<4;++k) {
    if(!FreeEdge(m,k))continue;
    const auto value=fp::FreeEdge(Position(in,m.nodes[k],length),Position(in,m.nodes[(k+1)%4],length),
        w.normal[4*main+k],fp::ReadyFloor());
    if(!fp::Finite(value))return {Status::NonfiniteResult,main};
    w.normal[4*main+k]=value;
  }
  return {Status::Ok};
}
TL_MATH_HOST_DEVICE inline void FillReference(Work w,std::size_t ref) {
  auto& output=w.references[ref];
  // Native LIMIT_CASE zeroes both slots above two boundaries; LBOUND retains
  // the actual count. Unassigned and zero-normal slots remain positive zero.
  if(output.boundary>2)return;
  for(int i=0;i<output.boundary;++i)output.bisector[i]=w.normal[w.slots[2*ref+std::size_t(i)]];
}
// All gathers finish before ANY Average invocation. Inactive scratch is never
// consumed or advertised as native-defined output.
TL_MATH_HOST_DEVICE inline void GatherNeighbor(const Input& in,Work w,std::size_t main) {
  if(!in.main_active[main])return;
  const auto& m=in.topology.mains[main];
  for(unsigned k=0;k<4;++k) {
    if(in.main_coefficients[main]<=0)w.neighbor[4*main+k]={};
    else if(m.neighbors[k]>0)
      w.neighbor[4*main+k]=w.normal[4*std::size_t(m.neighbors[k]-1)+std::size_t(m.neighbor_edges[k]-1)];
  }
}
TL_MATH_HOST_DEVICE inline Report Average(const Input& in,Work w,std::size_t main) {
  if(!in.main_active[main])return {Status::Ok};
  const auto& m=in.topology.mains[main];
  if(in.main_coefficients[main]<=0)for(unsigned k=0;k<4;++k)w.normal[4*main+k]={};
  for(unsigned k=0;k<4;++k)if(m.neighbors[k]>0) {
    const auto value=fp::Normalize(fp::Add(w.normal[4*main+k],w.neighbor[4*main+k]),fp::ReadyFloor());
    if(!fp::Finite(value))return {Status::NonfiniteResult,main};
    w.normal[4*main+k]=value;
  }
  return {Status::Ok};
}
} // namespace tlfea::contact::radioss_type25::current_normals::detail
