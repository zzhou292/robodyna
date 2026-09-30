// SPDX-License-Identifier: AGPL-3.0-or-later
#include "NodalDomainIdentity.h"
#include "../../lib_utils/BoundedArena.h"
#include "../../lib_utils/SourceIdentityIndex.h"
#include <cmath>
#include <new>

namespace tl::fea {
struct NodalNodeDomain::Impl {
  std::uint64_t source_instance_id=0;
  util::HostArena arena;
  NodalDomainNode* nodes=nullptr;
  util::SourceIdentityIndex<0> index;
  std::size_t count=0;
  std::size_t bytes=0;
};
NodalDomainReport NodalNodeDomain::Initialize(const NodalDomainInput& input,NodalDomainLimits limits) noexcept {
  using S=NodalDomainStatus;
  if(impl_) return {S::AlreadyInitialized,"Node domain is immutable"};
  const auto hard=NodalDomainLimits::Vehicle();
  if(!limits.max_nodes||limits.max_nodes>hard.max_nodes||!limits.max_host_bytes||
      limits.max_host_bytes>hard.max_host_bytes||!input.node_count||input.node_count>limits.max_nodes) {
    return {S::ResourceLimit,"Node domain count or limits exceed bounded scope"};
  }
  util::BoundedArenaLayout arena(limits.max_host_bytes),budget(limits.max_host_bytes);
  util::ArenaRegion nodes,ignored;
  if(!arena.Append<NodalDomainNode>(input.node_count,nodes)||
      !budget.Append<unsigned char>(sizeof(NodalNodeDomain)+sizeof(Impl)+64,ignored)||
      !budget.Append<unsigned char>(arena.bytes(),ignored)||
      !budget.Append<unsigned char>(decltype(Impl::index)::Storage::ExtraBytes(input.node_count),ignored)) {
    return {S::ResourceLimit,"Complete node domain payload exceeds cap"};
  }
  if(!input.source_instance_id||!nodal_domain_detail::ValidRange(input.nodes,input.node_count)) {
    return {S::InvalidInput,"Node domain source identity or borrowed range is invalid"};
  }
  try {
    auto next=std::make_shared<Impl>();
    if(!next->arena.Initialize(arena.bytes())||!(next->nodes=next->arena.Construct<NodalDomainNode>(nodes))) {
      return {S::ResourceLimit,"Node domain arena allocation failed"};
    }
    next->index.Prepare(input.node_count,[&](std::size_t n) {
      return input.nodes[n].source_id;
    });
    for(std::size_t n=0;n<input.node_count;++n) {
      const auto& node=input.nodes[n];
      if(!node.source_id||!std::isfinite(node.position.x)||
          !std::isfinite(node.position.y)||!std::isfinite(node.position.z)) {
        return {S::InvalidInput,"Node domain requires a positive NID and finite coordinates",n};
      }
      if(next->index.First(node.source_id)!=n) {
        return {S::DuplicateIdentity,"Source NID is repeated in the declared owner order",n};
      }
      next->nodes[n]=node;
    }
    next->source_instance_id=input.source_instance_id;
    next->count=input.node_count;
    next->bytes=budget.bytes();
    impl_=std::move(next);
    return {};
  } catch(const std::bad_alloc&) {
    return {S::ResourceLimit,"Node domain startup allocation failed"};
  }
}
std::uint64_t NodalNodeDomain::source_instance_id() const noexcept {return impl_?impl_->source_instance_id:0;}
std::size_t NodalNodeDomain::node_count() const noexcept {return impl_?impl_->count:0;}
tl::util::ConstView<NodalDomainNode> NodalNodeDomain::nodes() const noexcept {
  static const NodalDomainNode empty;
  return {impl_?impl_->nodes:&empty,node_count()};
}
std::size_t NodalNodeDomain::Find(std::uint64_t id) const noexcept {return impl_?impl_->index.First(id):SIZE_MAX;}
bool NodalNodeDomain::SharesStorage(const NodalNodeDomain& other) const noexcept {return impl_&&impl_==other.impl_;}
bool NodalNodeDomain::Matches(const NodalNodeDomain& other) const noexcept {
  if(!impl_||!other.impl_) return false;
  if(SharesStorage(other)) return true;
  if(source_instance_id()!=other.source_instance_id()||node_count()!=other.node_count()) return false;
  for(std::size_t n=0;n<node_count();++n) {
    const auto& a=nodes()[n];
    const auto& b=other.nodes()[n];
    if(a.source_id!=b.source_id||!nodal_domain_detail::SamePosition(a.position,b.position)) return false;
  }
  return true;
}
std::size_t NodalNodeDomain::owned_payload_bytes() const noexcept {return impl_?impl_->bytes:sizeof(NodalNodeDomain);}
std::size_t NodalNodeDomain::startup_payload_bytes() const noexcept {return owned_payload_bytes();}
} // namespace tl::fea
