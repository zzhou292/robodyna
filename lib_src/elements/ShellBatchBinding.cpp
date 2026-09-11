// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchBinding.h"
#include "ShellBindingIdentityIndex.h"
#include "qeph/QephStartup.h"
#include "t3/T3Startup.h"
#include <cstring>
#include <new>

namespace tl::fea {
namespace {
using namespace shell_binding_detail;
static_assert(sizeof(double)==sizeof(std::uint64_t)&&std::numeric_limits<double>::is_iec559,
              "Shell inventory requires the qualified binary64 representation");
static_assert(sizeof(std::size_t)<=sizeof(std::uint64_t),"Global indices fit the identity words");

std::uint64_t Bits(double value) noexcept {
  std::uint64_t result=0;
  std::memcpy(&result,&value,sizeof(result));
  return result;
}
bool SamePosition(tl::math::Vec3 a,tl::math::Vec3 b) noexcept {
  return Bits(a.x)==Bits(b.x)&&Bits(a.y)==Bits(b.y)&&Bits(a.z)==Bits(b.z);
}
ShellBindingReport Error(ShellBindingStatus status,const char* message,
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
bool Positive(double value) noexcept { return tl::math::Finite(value)&&value>0; }
bool AddMass(ShellBindingMass& sum,const ShellBindingMass& term) noexcept {
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
template<class Input,std::size_t N,class Words>
void AppendInventory(Words& words,
    std::size_t& cursor,std::uint64_t family,const Input& input,
    const std::array<std::size_t,N>& indices,std::uint64_t source_id,bool legacy) noexcept {
  words[cursor++]=family; words[cursor++]=N;
  if(!legacy) words[cursor++]=source_id;
  for(std::size_t i=0;i<N;++i) {
    words[cursor++]=indices[i]; words[cursor++]=input.node_ids[i];
    words[cursor++]=Bits(input.position[i].x); words[cursor++]=Bits(input.position[i].y);
    words[cursor++]=Bits(input.position[i].z);
  }
  words[cursor++]=Bits(input.density); words[cursor++]=Bits(input.thickness);
  words[cursor++]=Bits(input.young_modulus); words[cursor++]=Bits(input.poisson_ratio);
  words[cursor++]=static_cast<std::uint64_t>(input.placement);
}
} // namespace

const qeph::ReferenceData& ShellBatchBinding::qeph_reference(std::size_t i) const noexcept {
  static const qeph::ReferenceData empty;
  return prepared_&&i<data_.qeph_count?data_.qeph[i].reference:empty;
}
const t3::ReferenceData& ShellBatchBinding::t3_reference(std::size_t i) const noexcept {
  static const t3::ReferenceData empty;
  return prepared_&&i<data_.t3_count?data_.t3[i].reference:empty;
}
const std::array<std::size_t,4>& ShellBatchBinding::qeph_nodes(std::size_t i) const noexcept {
  static const std::array<std::size_t,4> empty{};
  return prepared_&&i<data_.qeph_count?data_.qeph[i].nodes:empty;
}
const std::array<std::size_t,3>& ShellBatchBinding::t3_nodes(std::size_t i) const noexcept {
  static const std::array<std::size_t,3> empty{};
  return prepared_&&i<data_.t3_count?data_.t3[i].nodes:empty;
}
std::uint64_t ShellBatchBinding::qeph_source_id(std::size_t i) const noexcept {
  return prepared_&&i<data_.qeph_count?data_.qeph[i].source_id:0;
}
std::uint64_t ShellBatchBinding::t3_source_id(std::size_t i) const noexcept {
  return prepared_&&i<data_.t3_count?data_.t3[i].source_id:0;
}
const qeph::ReferenceData& ShellBatchBinding::qeph_reference() const noexcept {
  return qeph_reference(data_.qeph_count==1&&data_.t3_count==1?0:NoShellBindingNode);
}
const t3::ReferenceData& ShellBatchBinding::t3_reference() const noexcept {
  return t3_reference(data_.qeph_count==1&&data_.t3_count==1?0:NoShellBindingNode);
}
const std::array<std::size_t,4>& ShellBatchBinding::qeph_nodes() const noexcept {
  return qeph_nodes(data_.qeph_count==1&&data_.t3_count==1?0:NoShellBindingNode);
}
const std::array<std::size_t,3>& ShellBatchBinding::t3_nodes() const noexcept {
  return t3_nodes(data_.qeph_count==1&&data_.t3_count==1?0:NoShellBindingNode);
}

ShellBindingReport ShellBatchBinding::Initialize(const ShellBatchBindingInput& input) noexcept {
  if(prepared_) return Error(ShellBindingStatus::AlreadyInitialized,"Shell binding is immutable after initialization");
  if(input.node_count<4||input.node_count>MaxShellBindingNodes)
    return Error(ShellBindingStatus::InvalidInput,"One Q4 and one T3 require 4 to 7 covered nodes");
  const ShellQephBindingInput q{input.qeph,input.qeph_nodes,0};
  const ShellT3BindingInput t{input.t3,input.t3_nodes,0};
  return InitializeImpl({&q,&t,1,1,input.node_count},true,
      {MaxShellCollectionParents,MaxShellCollectionNodes,4*1024*1024},false);
}
ShellBindingReport ShellBatchBinding::Initialize(const ShellBatchCollectionInput& input) noexcept {
  return InitializeImpl(input,false,
      {MaxShellCollectionParents,MaxShellCollectionNodes,4*1024*1024},false);
}

ShellBindingReport ShellBatchBinding::Initialize(const ShellBatchCollectionInput& input,
    const ShellHostBindingLimits& limits) noexcept {
  return InitializeImpl(input,false,limits,true);
}
std::size_t ShellBatchBinding::host_bytes() const noexcept {
  return sizeof(*this)+data_.qeph.backing_bytes()+data_.t3.backing_bytes()+
    data_.nodes.backing_bytes()+data_.inventory.backing_bytes();
}
ShellBindingReport ShellBatchBinding::InitializeImpl(const ShellBatchCollectionInput& input,bool legacy,
    const ShellHostBindingLimits& limits,bool expanded) noexcept {
  if(prepared_) return Error(ShellBindingStatus::AlreadyInitialized,"Shell binding is immutable after initialization");
  // Check counts before arithmetic or borrowed-range access. No caller storage
  // is read for a null/mismatched/oversized range.
  const bool vehicle=limits.max_parents>MaxShellHostParents||limits.max_nodes>MaxShellHostNodes;
  const auto parent_bound=vehicle?MaxVehicleShellBindingParents:MaxShellHostParents;
  const auto node_bound=vehicle?MaxVehicleShellBindingNodes:MaxShellHostNodes;
  if(input.node_count<3||input.node_count>node_bound||
     input.qeph_count>parent_bound||input.t3_count>parent_bound||
     input.qeph_count+input.t3_count==0||
     input.qeph_count+input.t3_count>parent_bound||
     (input.qeph_count==0)!=(input.qeph==nullptr)||
     (input.t3_count==0)!=(input.t3==nullptr))
    return Error(ShellBindingStatus::InvalidInput,"Invalid bounded typed collection ranges or node count");
  if(limits.max_parents>MaxVehicleShellBindingParents||limits.max_nodes>MaxVehicleShellBindingNodes||
     input.node_count>limits.max_nodes||input.qeph_count+input.t3_count>limits.max_parents)
    return Error(expanded?ShellBindingStatus::ResourceLimit:ShellBindingStatus::InvalidInput,
                 "Collection exceeds host admission limits");
  const auto words=legacy?ShellBatchInventory::WordCount:4+28*input.qeph_count+23*input.t3_count;
  const auto bytes=sizeof(*this)+decltype(data_.qeph)::ExtraBytes(input.qeph_count)+
    decltype(data_.t3)::ExtraBytes(input.t3_count)+decltype(data_.nodes)::ExtraBytes(input.node_count)+
    decltype(data_.inventory.words_)::ExtraBytes(words);
  if(bytes>limits.max_owned_bytes)
    return Error(ShellBindingStatus::ResourceLimit,"Owned host binding exceeds byte admission");
  if(limits.max_owned_bytes>MaxVehicleShellBindingOwnedBytes||
     limits.max_startup_scratch_bytes>MaxVehicleShellBindingScratchBytes||
     ScratchBytes(input.qeph_count,input.t3_count,input.node_count,legacy)>limits.max_startup_scratch_bytes)
    return Error(ShellBindingStatus::ResourceLimit,"Host binding exceeds bounded startup scratch or byte limits");
  try { return Build(input,legacy); }
  catch(const std::bad_alloc&) {
    return Error(ShellBindingStatus::ResourceLimit,"Host binding startup allocation failed");
  }
}
ShellBindingReport ShellBatchBinding::Build(const ShellBatchCollectionInput& input,bool legacy) {
  auto at=[](ShellBindingReport report,std::size_t parent) {
    report.parent_index=parent; return report;
  };
  ParentIdentityIndex parent_ids;
  if(!legacy) parent_ids.Prepare(input.qeph_count+input.t3_count,[&](std::size_t i) {
    return i<input.qeph_count?input.qeph[i].source_parent_id:input.t3[i-input.qeph_count].source_parent_id;
  });
  auto parent_identity=[&](std::uint64_t id,ShellBindingFamily family,std::size_t occurrence) {
    if(!legacy) {
      if(id==0) return Error(ShellBindingStatus::InvalidParentIdentity,"Collection parent ID is zero",family);
      if(parent_ids.First(id)!=occurrence)
        return Error(ShellBindingStatus::InvalidParentIdentity,"Collection parent ID is repeated",family);
    }
    return ShellBindingReport{};
  };
  // Preserve the pair's ordering: all connectivity checks, all native startup,
  // all identities/coverage, then native mass reduction and final publication.
  for(std::size_t i=0;i<input.qeph_count;++i) {
    auto report=CheckConnectivity(input.qeph[i].nodes,input.node_count,ShellBindingFamily::Qeph);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
    report=parent_identity(input.qeph[i].source_parent_id,ShellBindingFamily::Qeph,i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<input.t3_count;++i) {
    auto report=CheckConnectivity(input.t3[i].nodes,input.node_count,ShellBindingFamily::T3);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
    report=parent_identity(input.t3[i].source_parent_id,ShellBindingFamily::T3,input.qeph_count+i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  Data next;
  next.qeph.Resize(input.qeph_count); next.t3.Resize(input.t3_count);
  next.nodes.Resize(input.node_count);
  next.inventory.words_.Resize(legacy?ShellBatchInventory::WordCount:4+28*input.qeph_count+23*input.t3_count);
  next.qeph_count=input.qeph_count; next.t3_count=input.t3_count; next.node_count=input.node_count;
  for(std::size_t i=0;i<input.qeph_count;++i) {
    auto& parent=next.qeph[i];
    const auto status=qeph::InitializeReference(input.qeph[i].reference,parent.reference);
    if(status!=qeph::Status::kSuccess) {
      auto report=Error(ShellBindingStatus::InvalidQephReference,"QEPH startup rejected its typed input",ShellBindingFamily::Qeph);
      report.qeph_status=status; return at(report,i);
    }
    parent.nodes=input.qeph[i].nodes; parent.source_id=input.qeph[i].source_parent_id;
  }
  for(std::size_t i=0;i<input.t3_count;++i) {
    auto& parent=next.t3[i];
    const auto status=t3::InitializeReference(input.t3[i].reference,parent.reference);
    if(status!=t3::Status::kSuccess) {
      auto report=Error(ShellBindingStatus::InvalidT3Reference,"T3 startup rejected its typed input",ShellBindingFamily::T3);
      report.t3_status=status; return at(report,i);
    }
    parent.nodes=input.t3[i].nodes; parent.source_id=input.t3[i].source_parent_id;
  }
  NodeSeen seen; seen.Resize(next.node_count);
  NodeIdentityIndex node_ids;
  node_ids.Prepare(4*next.qeph_count+3*next.t3_count,[&](std::size_t i)->std::uint64_t {
    if(i<4*next.qeph_count) return next.qeph[i/4].reference.input.node_ids[i%4];
    const auto t=i-4*next.qeph_count;
    return next.t3[t/3].reference.input.node_ids[t%3];
  });
  for(std::size_t i=0;i<next.qeph_count;++i) {
    const auto& parent=next.qeph[i];
    const auto report=RegisterNodes(parent.reference.input,parent.nodes,ShellBindingFamily::Qeph,
        next.nodes.data(),seen.data(),node_ids,4*i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<next.t3_count;++i) {
    const auto& parent=next.t3[i];
    const auto report=RegisterNodes(parent.reference.input,parent.nodes,ShellBindingFamily::T3,
        next.nodes.data(),seen.data(),node_ids,4*next.qeph_count+3*i);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t n=0;n<next.node_count;++n) if(!seen[n])
    return Error(ShellBindingStatus::InvalidConnectivity,"Declared global node is uncovered",
                 ShellBindingFamily::None,NoShellBindingNode,n);
  for(std::size_t i=0;i<next.qeph_count;++i) {
    const auto& parent=next.qeph[i];
    const auto report=Accumulate(parent.reference,parent.nodes,ShellBindingFamily::Qeph,next.nodes.data(),next.totals);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  for(std::size_t i=0;i<next.t3_count;++i) {
    const auto& parent=next.t3[i];
    const auto report=Accumulate(parent.reference,parent.nodes,ShellBindingFamily::T3,next.nodes.data(),next.totals);
    if(report.status!=ShellBindingStatus::Success) return at(report,i);
  }
  static_assert(ShellBatchInventory::WordCount==2+(2+5*4+5)+(2+5*3+5),"Complete placed pair inventory");
  static_assert(ShellBatchInventory::Capacity>=4+MaxShellCollectionParents*(3+5*4+5),"Complete collection inventory");
  auto& words=next.inventory.words_;
  std::size_t cursor=0;
  words[cursor++]=legacy?3:4; // In-process encoding only, never a file schema.
  words[cursor++]=next.node_count;
  if(!legacy) { words[cursor++]=next.qeph_count; words[cursor++]=next.t3_count; }
  for(std::size_t i=0;i<next.qeph_count;++i) {
    const auto& parent=next.qeph[i];
    AppendInventory(words,cursor,4,parent.reference.input,parent.nodes,parent.source_id,legacy);
  }
  for(std::size_t i=0;i<next.t3_count;++i) {
    const auto& parent=next.t3[i];
    AppendInventory(words,cursor,3,parent.reference.input,parent.nodes,parent.source_id,legacy);
  }
  next.inventory.word_count_=cursor;
  data_=next; prepared_=true;
  return {};
}
} // namespace tl::fea
