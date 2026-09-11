// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "TiedClassificationInternal.h"
#include <algorithm>

namespace tl::constraints::tied_shell::classification_detail {
inline bool Selected(const ClassificationInterface& r) noexcept {
  return r.native_type==2 && (r.level==27 || r.level==28);
}
inline bool Contains(ClassificationNodes nodes,std::uint32_t node) noexcept {
  for(std::size_t i=0;i<nodes.count;++i) if(nodes.data[i]==node) return true;
  return false;
}
template<class Data> Report RunClassification(const ClassificationInput& in,Data& out) {
  const auto count=out.node_count;
  auto penalty=std::make_unique<std::int32_t[]>(count);
  auto seen=std::make_unique<std::int32_t[]>(count);
  auto midpoint=std::make_unique<std::int32_t[]>(count);
  auto scratch=std::make_unique<NativeKinematics[]>(count);
  // ITAGSL2 first marks all occurrences before any ITF table mutation.
  for(std::size_t n=0;n<in.interfaces.count;++n) {
    const auto& r=in.interfaces.data[n];
    if(!Selected(r)) continue;
    for(std::size_t i=0;i<r.slaves.count;++i) {
      const auto node=r.slaves.data[i];
      const auto code=out.nodes[node].kinematics.conditions;
      int condition=seen[node];
      for(const int kind : {1,2,8,128,16,32,64,256,512,1024,2048,4096})
        condition+=Decode(kind,code,out.itf);
      if(in.cyclic_tags.count) condition+=in.cyclic_tags.data[node];
      if(condition!=0) penalty[node]=1;
      seen[node]=1;
    }
  }
  for(std::size_t n=0;n<in.sections.count;++n) {
    const auto& section=in.sections.data[n];
    if(section.native_type!=100 && section.native_type!=101) continue;
    for(std::size_t i=0;i<section.nodes.count;++i) {
      const auto node=section.nodes.data[i];
      if(seen[node]==1) penalty[node]=1;
    }
  }
  for(std::size_t n=0;n<in.interfaces.count;++n) {
    const auto& r=in.interfaces.data[n];
    if(r.native_type!=2) continue;
    if(r.level!=0 && r.level!=1 && r.level!=27 && r.level!=28) continue;
    for(std::size_t i=0;i<r.masters.count;++i) penalty[r.masters.data[i]]=1;
  }
  if(in.tetra_edges.count) {
    for(std::size_t n=0;n<in.interfaces.count;++n) {
      const auto& r=in.interfaces.data[n];
      if(!Selected(r)) continue;
      for(std::size_t i=0;i<r.slaves.count;++i) {
        const auto node=r.slaves.data[i];
        const auto tag=in.tetra_tags.data[node];
        if(!tag || penalty[node]==1) continue;
        const auto index=static_cast<std::size_t>(tag<0 ? -static_cast<std::int64_t>(tag) : tag)-1;
        const auto edge=in.tetra_edges.data[index];
        if(!Contains(r.slaves,edge.first_corner) || !Contains(r.slaves,edge.second_corner))
          penalty[node]=1;
      }
    }
    for(std::size_t i=0;i<in.tetra_edges.count;++i)
      midpoint[in.tetra_edges.data[i].midpoint]=static_cast<std::int32_t>(i+1);
    for(std::size_t n=0;n<in.interfaces.count;++n) {
      const auto& r=in.interfaces.data[n];
      if(!Selected(r)) continue;
      for(std::size_t i=0;i<r.slaves.count;++i) {
        const auto node=r.slaves.data[i];
        if(penalty[node]!=1 && midpoint[node]==0) midpoint[node]=-1;
      }
      for(std::size_t i=0;i<r.masters.count;++i) {
        auto& tag=midpoint[r.masters.data[i]];
        if(tag>0) {
          if(in.tetra_edges.count>static_cast<std::size_t>(INT_MAX-tag))
            return Fail(Status::NativeDomain,n,i);
          tag+=static_cast<std::int32_t>(in.tetra_edges.count);
        }
      }
    }
    for(std::size_t i=0;i<in.tetra_edges.count;++i) {
      const auto edge=in.tetra_edges.data[i];
      if(midpoint[edge.midpoint]>static_cast<std::int32_t>(in.tetra_edges.count) || penalty[edge.midpoint]==1) {
        if(midpoint[edge.first_corner]<0) penalty[edge.first_corner]=1;
        if(midpoint[edge.second_corner]<0) penalty[edge.second_corner]=1;
      }
    }
  }
  for(std::size_t n=0;n<in.rbe3_members.count;++n) {
    const auto members=in.rbe3_members.data[n];
    for(std::size_t i=0;i<members.count;++i) penalty[members.data[i]]=1;
  }
  for(std::size_t i=0;i<in.rbe2_nodes.count;++i) penalty[in.rbe2_nodes.data[i]]=1;

  out.interface_count=in.interfaces.count;
  out.interfaces=std::make_unique<ClassifiedInterface[]>(out.interface_count);
  out.irupt=std::make_unique<std::int32_t[]>(out.slave_count);
  out.slaves=std::make_unique<std::uint32_t[]>(out.slave_count);
  std::size_t offset=0;
  for(std::size_t n=0;n<in.interfaces.count;++n) {
    const auto& r=in.interfaces.data[n];
    out.interfaces[n]={r.source_id,r.native_type,r.level,offset,r.slaves.count,Selected(r)};
    for(std::size_t i=0;i<r.slaves.count;++i) out.slaves[offset+i]=r.slaves.data[i];
    if(Selected(r)) {
      for(std::size_t i=0;i<r.slaves.count;++i) {
        const auto node=r.slaves.data[i];
        auto& k=out.nodes[node].kinematics;
        if(penalty[node]==1) {
          out.irupt[offset+i]=1;
          out.itf[k.conditions]=0;
          ++out.penalty_warnings;
        } else {
          for(int direction=0;direction<6;++direction)
            if(!KinSet(2,direction,k,scratch[node],out.itf,out.kinset_warnings))
              return Fail(Status::NativeDomain,n,i);
        }
      }
    }
    offset+=r.slaves.count;
  }
  out.phase=ClassificationPhase::InterfaceTaggedBeforeKinChk;
  return {};
}
template<class Data> Report RunRigidRegistration(const RigidRegistrationInput& in,Data& out) {
  auto scratch=std::make_unique<NativeKinematics[]>(out.node_count);
  if(in.native_iddlevel==0) {
    const int kind=in.native_ikrem==0 ? 8 : 128;
    for(std::size_t n=0;n<in.groups.count;++n) {
      const auto members=in.groups.data[n].members;
      for(std::size_t i=0;i<members.count;++i) {
        const auto node=members.data[i];
        for(int direction=0;direction<6;++direction)
          if(!KinSet(kind,direction,out.nodes[node].kinematics,scratch[node],out.itf,out.kinset_warnings))
            return Fail(Status::NativeDomain,n,i);
      }
    }
  }
  out.phase=ClassificationPhase::RigidMembersRegistered;
  return {};
}
} // namespace tl::constraints::tied_shell::classification_detail
