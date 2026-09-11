// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ShellNodeMap.h"
#include "NodalDomainIdentity.h"
#include "../../lib_utils/BoundedArena.h"
#include <new>

namespace tl::fea {
struct ShellNodeMap::Impl {
  Impl(const ShellBatchBinding& s,const NodalNodeDomain& d):shells(s),domain(d) {}
  ShellBatchBinding shells;
  NodalNodeDomain domain;
  util::HostArena arena;
  std::size_t* map=nullptr;
  std::size_t bytes=0;
  bool identity=false;
};
NodalDomainReport ShellNodeMap::Initialize(const ShellBatchBinding& shells,const NodalNodeDomain& domain,
    ShellNodeMapLimits limits) noexcept {
  using S=NodalDomainStatus;
  if(impl_) return {S::AlreadyInitialized,"Shell node map is immutable"};
  if(!shells.prepared()||!domain.prepared()) {
    return {S::InvalidInput,"Prepared shell and node domain inputs are required"};
  }
  const auto hard=ShellNodeMapLimits::Vehicle();
  if(!limits.max_nodes||limits.max_nodes>hard.max_nodes||!limits.max_host_bytes||
      limits.max_host_bytes>hard.max_host_bytes||shells.node_count()>limits.max_nodes||
      domain.node_count()>limits.max_nodes) {
    return {S::ResourceLimit,"Shell map node counts or limits exceed bounded scope"};
  }
  const auto shell_bytes=shells.host_bytes();
  const auto domain_bytes=domain.owned_payload_bytes();
  if(shell_bytes<sizeof(ShellBatchBinding)||domain_bytes<sizeof(NodalNodeDomain)) {
    return {S::ResourceLimit,"Retained immutable producer payload is inconsistent"};
  }
  util::BoundedArenaLayout arena(limits.max_host_bytes),budget(limits.max_host_bytes);
  util::ArenaRegion map,ignored;
  if(!arena.Append<std::size_t>(shells.node_count(),map)||
      !budget.Append<unsigned char>(sizeof(ShellNodeMap)+sizeof(Impl)+64,ignored)||
      !budget.Append<unsigned char>(arena.bytes(),ignored)||
      !budget.Append<unsigned char>(shell_bytes-sizeof(ShellBatchBinding),ignored)||
      !budget.Append<unsigned char>(domain_bytes-sizeof(NodalNodeDomain),ignored)) {
    return {S::ResourceLimit,"Shell map and retained producer payload exceed cap"};
  }
  try {
    auto next=std::make_shared<Impl>(shells,domain);
    if(!next->arena.Initialize(arena.bytes())||!(next->map=next->arena.Construct<std::size_t>(map))) {
      return {S::ResourceLimit,"Shell map arena allocation failed"};
    }
    next->identity=shells.node_count()==domain.node_count();
    for(std::size_t local=0;local<shells.node_count();++local) {
      const auto& node=shells.nodes()[local];
      const auto owner=domain.Find(node.source_id);
      if(owner==SIZE_MAX) {
        return {S::MissingSource,"Shell source NID is absent from declared owner domain",local};
      }
      if(!nodal_domain_detail::SamePosition(node.position,domain.nodes()[owner].position)) {
        return {S::PositionMismatch,"Shell and domain source coordinate bits differ",local};
      }
      // Both prepared inputs enforce unique NIDs, so this exact lookup is injective.
      next->map[local]=owner;
      next->identity=next->identity&&owner==local;
    }
    next->bytes=budget.bytes();
    impl_=std::move(next);
    return {};
  } catch(const std::bad_alloc&) {
    return {S::ResourceLimit,"Shell map startup allocation failed"};
  }
}
const ShellBatchBinding* ShellNodeMap::shells() const noexcept {return impl_?&impl_->shells:nullptr;}
const NodalNodeDomain* ShellNodeMap::domain() const noexcept {return impl_?&impl_->domain:nullptr;}
std::size_t ShellNodeMap::shell_node_count() const noexcept {return impl_?impl_->shells.node_count():0;}
std::size_t ShellNodeMap::owner_node_count() const noexcept {return impl_?impl_->domain.node_count():0;}
tl::util::ConstView<std::size_t> ShellNodeMap::mapping() const noexcept {
  static const std::size_t empty=SIZE_MAX;
  return {impl_?impl_->map:&empty,shell_node_count()};
}
std::size_t ShellNodeMap::owner_index(std::size_t local) const noexcept {
  return local<shell_node_count()?impl_->map[local]:SIZE_MAX;
}
bool ShellNodeMap::identity_map() const noexcept {return impl_&&impl_->identity;}
bool ShellNodeMap::Matches(const ShellBatchBinding& shells,const NodalNodeDomain& domain) const noexcept {
  return impl_&&shells.prepared()&&impl_->shells.inventory()==shells.inventory()&&impl_->domain.Matches(domain);
}
bool ShellNodeMap::Matches(const ShellNodeMap& other) const noexcept {
  return impl_&&other.impl_&&(impl_==other.impl_||Matches(other.impl_->shells,other.impl_->domain));
}
std::size_t ShellNodeMap::owned_payload_bytes() const noexcept {return impl_?impl_->bytes:sizeof(ShellNodeMap);}
std::size_t ShellNodeMap::startup_payload_bytes() const noexcept {return owned_payload_bytes();}
} // namespace tl::fea
