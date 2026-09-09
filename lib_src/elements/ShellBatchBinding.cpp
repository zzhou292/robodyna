// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBatchBinding.h"
#include "qeph/QephStartup.h"
#include "t3/T3Startup.h"
#include <cstring>

namespace tl::fea {
namespace {
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
    ShellBindingFamily family,std::array<ShellBindingNode,MaxShellBindingNodes>& nodes,
    std::array<bool,MaxShellBindingNodes>& seen) noexcept {
  for(std::size_t i=0;i<N;++i) {
    const auto n=indices[i];
    const std::uint64_t id=input.node_ids[i]; // Widen QEPH, preserve all T3 bits.
    if(seen[n]) {
      if(nodes[n].source_id!=id)
        return Error(ShellBindingStatus::IdentityMismatch,"Shared source IDs differ",family,i,n);
      if(!SamePosition(nodes[n].position,input.position[i]))
        return Error(ShellBindingStatus::PositionMismatch,"Shared coordinate bits differ",family,i,n);
    } else {
      for(std::size_t other=0;other<MaxShellBindingNodes;++other)
        if(seen[other]&&nodes[other].source_id==id)
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
    ShellBindingFamily family,std::array<ShellBindingNode,MaxShellBindingNodes>& nodes,
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
template<class Input,std::size_t N>
void AppendInventory(std::array<std::uint64_t,ShellBatchInventory::WordCount>& words,
    std::size_t& cursor,std::uint64_t family,const Input& input,
    const std::array<std::size_t,N>& indices) noexcept {
  words[cursor++]=family; words[cursor++]=N;
  for(std::size_t i=0;i<N;++i) {
    words[cursor++]=indices[i]; words[cursor++]=input.node_ids[i];
    words[cursor++]=Bits(input.position[i].x); words[cursor++]=Bits(input.position[i].y);
    words[cursor++]=Bits(input.position[i].z);
  }
  words[cursor++]=Bits(input.density); words[cursor++]=Bits(input.thickness);
  words[cursor++]=Bits(input.young_modulus); words[cursor++]=Bits(input.poisson_ratio);
}
} // namespace

ShellBindingReport ShellBatchBinding::Initialize(const ShellBatchBindingInput& input) noexcept {
  if(prepared_) return Error(ShellBindingStatus::AlreadyInitialized,"Shell binding is immutable after initialization");
  if(input.node_count<4||input.node_count>MaxShellBindingNodes)
    return Error(ShellBindingStatus::InvalidInput,"One Q4 and one T3 require 4 to 7 covered nodes");
  auto report=CheckConnectivity(input.qeph_nodes,input.node_count,ShellBindingFamily::Qeph);
  if(report.status!=ShellBindingStatus::Success) return report;
  report=CheckConnectivity(input.t3_nodes,input.node_count,ShellBindingFamily::T3);
  if(report.status!=ShellBindingStatus::Success) return report;

  Data next;
  const auto q=qeph::InitializeReference(input.qeph,next.qeph);
  if(q!=qeph::Status::kSuccess) {
    report=Error(ShellBindingStatus::InvalidQephReference,"QEPH startup rejected its typed input",ShellBindingFamily::Qeph);
    report.qeph_status=q; return report;
  }
  const auto t=t3::InitializeReference(input.t3,next.t3);
  if(t!=t3::Status::kSuccess) {
    report=Error(ShellBindingStatus::InvalidT3Reference,"T3 startup rejected its typed input",ShellBindingFamily::T3);
    report.t3_status=t; return report;
  }
  next.qeph_nodes=input.qeph_nodes; next.t3_nodes=input.t3_nodes; next.node_count=input.node_count;
  std::array<bool,MaxShellBindingNodes> seen{};
  report=RegisterNodes(next.qeph.input,next.qeph_nodes,ShellBindingFamily::Qeph,next.nodes,seen);
  if(report.status!=ShellBindingStatus::Success) return report;
  report=RegisterNodes(next.t3.input,next.t3_nodes,ShellBindingFamily::T3,next.nodes,seen);
  if(report.status!=ShellBindingStatus::Success) return report;
  for(std::size_t n=0;n<next.node_count;++n) if(!seen[n])
    return Error(ShellBindingStatus::InvalidConnectivity,"Declared global node is uncovered",
                 ShellBindingFamily::None,NoShellBindingNode,n);
  report=Accumulate(next.qeph,next.qeph_nodes,ShellBindingFamily::Qeph,next.nodes,next.totals);
  if(report.status!=ShellBindingStatus::Success) return report;
  report=Accumulate(next.t3,next.t3_nodes,ShellBindingFamily::T3,next.nodes,next.totals);
  if(report.status!=ShellBindingStatus::Success) return report;

  static_assert(ShellBatchInventory::WordCount==2+(2+5*4+4)+(2+5*3+4),"Complete native identity inventory");
  auto& words=next.inventory.words_;
  std::size_t cursor=0;
  words[cursor++]=1; // In-process encoding discriminator only, not a file schema.
  words[cursor++]=next.node_count;
  AppendInventory(words,cursor,4,next.qeph.input,next.qeph_nodes);
  AppendInventory(words,cursor,3,next.t3.input,next.t3_nodes);
  data_=next; prepared_=true;
  return {};
}
} // namespace tl::fea
