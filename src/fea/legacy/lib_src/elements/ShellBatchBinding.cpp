// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellBindingInventory.h"
#include <new>

namespace tl::fea {
using namespace shell_binding_detail;

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
  return qeph_reference(data_.qeph_count==1&&data_.t3_count==1&&data_.qbat_count==0?0:NoShellBindingNode);
}
const t3::ReferenceData& ShellBatchBinding::t3_reference() const noexcept {
  return t3_reference(data_.qeph_count==1&&data_.t3_count==1&&data_.qbat_count==0?0:NoShellBindingNode);
}
const std::array<std::size_t,4>& ShellBatchBinding::qeph_nodes() const noexcept {
  return qeph_nodes(data_.qeph_count==1&&data_.t3_count==1&&data_.qbat_count==0?0:NoShellBindingNode);
}
const std::array<std::size_t,3>& ShellBatchBinding::t3_nodes() const noexcept {
  return t3_nodes(data_.qeph_count==1&&data_.t3_count==1&&data_.qbat_count==0?0:NoShellBindingNode);
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
    data_.nodes.backing_bytes()+data_.inventory.backing_bytes()+data_.qbat.backing_bytes();
}
ShellBindingReport ShellBatchBinding::InitializeImpl(const ShellBatchCollectionInput& input,bool legacy,
    const ShellHostBindingLimits& limits,bool expanded,
    const ShellQbatBindingInput* qbat,std::size_t qbat_count) noexcept {
  if(prepared_) return Error(ShellBindingStatus::AlreadyInitialized,"Shell binding is immutable after initialization");
  // Check counts before arithmetic or borrowed-range access. No caller storage
  // is read for a null/mismatched/oversized range.
  const bool vehicle=limits.max_parents>MaxShellHostParents||limits.max_nodes>MaxShellHostNodes;
  const auto parent_bound=vehicle?MaxVehicleShellBindingParents:MaxShellHostParents;
  const auto node_bound=vehicle?MaxVehicleShellBindingNodes:MaxShellHostNodes;
  if(input.node_count<3||input.node_count>node_bound||
     input.qeph_count>parent_bound||input.t3_count>parent_bound||qbat_count>parent_bound||
     input.qeph_count+input.t3_count+qbat_count==0||
     input.qeph_count+input.t3_count+qbat_count>parent_bound||
     (input.qeph_count==0)!=(input.qeph==nullptr)||
     (input.t3_count==0)!=(input.t3==nullptr)||
     (qbat_count==0)!=(qbat==nullptr))
    return Error(ShellBindingStatus::InvalidInput,"Invalid bounded typed collection ranges or node count");
  if(limits.max_parents>MaxVehicleShellBindingParents||limits.max_nodes>MaxVehicleShellBindingNodes||
     input.node_count>limits.max_nodes||input.qeph_count+input.t3_count+qbat_count>limits.max_parents)
    return Error(expanded?ShellBindingStatus::ResourceLimit:ShellBindingStatus::InvalidInput,
                 "Collection exceeds host admission limits");
  const auto words=InventoryWords(input.qeph_count,input.t3_count,qbat_count,legacy);
  const auto bytes=sizeof(*this)+decltype(data_.qeph)::ExtraBytes(input.qeph_count)+
    decltype(data_.t3)::ExtraBytes(input.t3_count)+decltype(data_.nodes)::ExtraBytes(input.node_count)+
    decltype(data_.inventory.words_)::ExtraBytes(words)+decltype(data_.qbat)::ExtraBytes(qbat_count);
  if(bytes>limits.max_owned_bytes)
    return Error(ShellBindingStatus::ResourceLimit,"Owned host binding exceeds byte admission");
  if(limits.max_owned_bytes>MaxVehicleShellBindingOwnedBytes||
     limits.max_startup_scratch_bytes>MaxVehicleShellBindingScratchBytes||
     ScratchBytes(input.qeph_count,input.t3_count,input.node_count,legacy,qbat_count)>limits.max_startup_scratch_bytes)
    return Error(ShellBindingStatus::ResourceLimit,"Host binding exceeds bounded startup scratch or byte limits");
  // New formulation admission validates complete borrowed extents. Legacy
  // error ordering above remains unchanged, and no payload has been read.
  if(qbat_count&&(!ValidRange(input.qeph,input.qeph_count)||
      !ValidRange(input.t3,input.t3_count)||!ValidRange(qbat,qbat_count)))
    return Error(ShellBindingStatus::InvalidInput,"Invalid formulation collection range");
  try { return Build(input,legacy,qbat,qbat_count); }
  catch(const std::bad_alloc&) {
    return Error(ShellBindingStatus::ResourceLimit,"Host binding startup allocation failed");
  }
}
} // namespace tl::fea
