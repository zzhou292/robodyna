// SPDX-License-Identifier: AGPL-3.0-or-later
#pragma once
#include "ShellBatchBinding.h"
#include "ShellBindingIdentityIndex.h"
#include "../math/Quaternion.h"
#include <cstring>

namespace tl::fea::shell_binding_detail {
template<class T>
bool ValidRange(const T* pointer,std::size_t count) noexcept {
  if(!count) return pointer==nullptr;
  const auto address=reinterpret_cast<std::uintptr_t>(pointer);
  return pointer&&address%alignof(T)==0&&count<=SIZE_MAX/sizeof(T)&&
      address<=UINTPTR_MAX-count*sizeof(T);
}
static_assert(sizeof(double)==sizeof(std::uint64_t)&&std::numeric_limits<double>::is_iec559,
              "Shell inventory requires the qualified binary64 representation");
static_assert(sizeof(std::size_t)<=sizeof(std::uint64_t),"Global indices fit the identity words");

inline std::uint64_t Bits(double value) noexcept {
  std::uint64_t result=0;
  std::memcpy(&result,&value,sizeof(result));
  return result;
}
inline bool SamePosition(tl::math::Vec3 a,tl::math::Vec3 b) noexcept {
  return Bits(a.x)==Bits(b.x)&&Bits(a.y)==Bits(b.y)&&Bits(a.z)==Bits(b.z);
}
inline ShellBindingReport Error(ShellBindingStatus status,const char* message,
    ShellBindingFamily family=ShellBindingFamily::None,
    std::size_t local=NoShellBindingNode,std::size_t global=NoShellBindingNode) noexcept {
  ShellBindingReport result;
  result.status=status; result.message=message; result.family=family;
  result.local_node=local; result.global_node=global;
  return result;
}
template<std::size_t N>
ShellBindingReport CheckConnectivity(const std::array<std::size_t,N>& indices,
    std::size_t count,ShellBindingFamily family) noexcept {
  for(std::size_t i=0;i<N;++i) {
    if(indices[i]>=count)
      return Error(ShellBindingStatus::InvalidConnectivity,"Global node is out of range",family,i,indices[i]);
    for(std::size_t j=0;j<i;++j) if(indices[i]==indices[j])
      return Error(ShellBindingStatus::InvalidConnectivity,"Repeated native connectivity",family,i,indices[i]);
  }
  return {};
}
template<class Input,std::size_t N>
ShellBindingReport RegisterNodes(const Input& input,const std::array<std::size_t,N>& indices,
    ShellBindingFamily family,ShellBindingNode* nodes,
    bool* seen,const NodeIdentityIndex& identities,std::size_t first_occurrence) noexcept {
  for(std::size_t i=0;i<N;++i) {
    const auto n=indices[i];
    const std::uint64_t id=input.node_ids[i]; // Widen QEPH, preserve all T3 bits.
    if(seen[n]) {
      if(nodes[n].source_id!=id)
        return Error(ShellBindingStatus::IdentityMismatch,"Shared source IDs differ",family,i,n);
      if(!SamePosition(nodes[n].position,input.position[i]))
        return Error(ShellBindingStatus::PositionMismatch,"Shared coordinate bits differ",family,i,n);
    } else {
      if(identities.First(id)!=first_occurrence+i)
        return Error(ShellBindingStatus::IdentityMismatch,"Source ID maps to distinct global nodes",family,i,n);
      nodes[n].source_id=id; nodes[n].position=input.position[i]; seen[n]=true;
    }
  }
  return {};
}
inline bool Positive(double value) noexcept { return tl::math::Finite(value)&&value>0; }
inline bool AddMass(ShellBindingMass& sum,const ShellBindingMass& term) noexcept {
  if(!Positive(term.mass)||!Positive(term.isotropic_inertia)||
     !Positive(term.physical_inertia)||!Positive(term.added_inertia)) return false;
  sum.mass+=term.mass;
  sum.isotropic_inertia+=term.isotropic_inertia;
  sum.physical_inertia+=term.physical_inertia;
  sum.added_inertia+=term.added_inertia;
  return Positive(sum.mass)&&Positive(sum.isotropic_inertia)&&
         Positive(sum.physical_inertia)&&Positive(sum.added_inertia);
}
template<class Reference,std::size_t N>
ShellBindingReport Accumulate(const Reference& reference,const std::array<std::size_t,N>& indices,
    ShellBindingFamily family,ShellBindingNode* nodes,
    ShellBindingMass& totals) noexcept {
  for(std::size_t i=0;i<N;++i) {
    const ShellBindingMass term{reference.nodal_mass[i],reference.isotropic_inertia[i],
                               reference.physical_inertia[i],reference.added_inertia[i]};
    if(!AddMass(nodes[indices[i]].native,term)||!AddMass(totals,term))
      return Error(ShellBindingStatus::NonfiniteMass,"Native union mass or inertia sum is not representable",
                   family,i,indices[i]);
  }
  return {};
}
} // namespace tl::fea::shell_binding_detail
