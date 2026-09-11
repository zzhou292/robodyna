// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Type13ContributionChecks.h"
#include "../../lib_utils/BoundedArena.h"
#include <new>

namespace tl::fea {
struct Type13NodeContributions::Impl {
  Impl(const type13::Model& m,const NodalNodeDomain& d):model(m),domain(d) {}
  type13::Model model;
  NodalNodeDomain domain;
  util::HostArena arena;
  Type13NodeContribution* records=nullptr;
  std::size_t count=0,bytes=0;
};
NodalDomainReport Type13NodeContributions::Initialize(const type13::Model& model,const NodalNodeDomain& domain,
    Type13ContributionLimits limits) noexcept {
  using S=NodalDomainStatus;
  if(impl_) return {S::AlreadyInitialized,"TYPE13 node contributions are immutable"};
  if(!model.prepared()||!domain.prepared()) return {S::InvalidInput,"Prepared TYPE13 model and domain are required"};
  const Type13ContributionLimits hard;
  if(!limits.max_connections||limits.max_connections>hard.max_connections||
      !limits.max_nodes||limits.max_nodes>hard.max_nodes||!limits.max_host_bytes||
      limits.max_host_bytes>hard.max_host_bytes||model.connection_count()>limits.max_connections||
      domain.node_count()>limits.max_nodes) return {S::ResourceLimit,"TYPE13 contribution counts or caps exceed scope"};
  const auto model_bytes=model.owned_payload_bytes();
  const auto domain_bytes=domain.owned_payload_bytes();
  if(model_bytes<sizeof(type13::Model)||domain_bytes<sizeof(NodalNodeDomain)) {
    return {S::ResourceLimit,"TYPE13 retained producer payload is inconsistent"};
  }
  const auto count=2*model.connection_count(); // Bounded above before multiplication.
  util::BoundedArenaLayout arena(limits.max_host_bytes),budget(limits.max_host_bytes);
  util::ArenaRegion records,ignored;
  if(!arena.Append<Type13NodeContribution>(count,records)||
      !budget.Append<unsigned char>(sizeof(Type13NodeContributions)+sizeof(Impl)+64,ignored)||
      !budget.Append<unsigned char>(arena.bytes(),ignored)||
      !budget.Append<unsigned char>(model_bytes-sizeof(type13::Model),ignored)||
      !budget.Append<unsigned char>(domain_bytes-sizeof(NodalNodeDomain),ignored)) {
    return {S::ResourceLimit,"TYPE13 records and complete retained payload exceed cap"};
  }
  if(model.source_instance_id()!=domain.source_instance_id()||model.global_node_count()!=domain.node_count()) {
    return {S::InvalidInput,"TYPE13 model source instance or owner extent differs from domain"};
  }
  const auto checked=type13_contribution_detail::CheckNodes(model,domain);
  if(!checked) return checked;
  try {
    auto next=std::make_shared<Impl>(model,domain);
    if(!next->arena.Initialize(arena.bytes())||!(next->records=next->arena.Construct<Type13NodeContribution>(records))) {
      return {S::ResourceLimit,"TYPE13 contribution arena allocation failed"};
    }
    for(std::size_t e=0;e<model.connection_count();++e) {
      const auto& connection=model.connections()[e];
      const auto* startup=model.startup(e);
      for(unsigned local=0;local<2;++local) {
        type13::EndpointContribution value;
        if(!startup||!model.Endpoint(e,local,value)) return {S::InvalidInput,"TYPE13 prepared endpoint query failed"};
        if(!nodal_domain_detail::SamePosition(startup->reference.position_m[local],domain.nodes()[value.global_node].position)) {
          return {S::PositionMismatch,"TYPE13 prepared SI endpoint coordinate bits differ from domain",connection.node[local]};
        }
        next->records[2*e+local]={e,local,value};
      }
      const auto orientation=type13_contribution_detail::CheckOrientation(model,domain,connection.node[2]);
      if(!orientation) return orientation;
    }
    next->count=count;
    next->bytes=budget.bytes();
    impl_=std::move(next);
    return {};
  } catch(const std::bad_alloc&) {
    return {S::ResourceLimit,"TYPE13 contribution startup allocation failed"};
  }
}
const type13::Model* Type13NodeContributions::model() const noexcept {return impl_?&impl_->model:nullptr;}
const NodalNodeDomain* Type13NodeContributions::domain() const noexcept {return impl_?&impl_->domain:nullptr;}
std::size_t Type13NodeContributions::record_count() const noexcept {return impl_?impl_->count:0;}
tl::util::ConstView<Type13NodeContribution> Type13NodeContributions::records() const noexcept {
  static const Type13NodeContribution empty;
  return {impl_?impl_->records:&empty,record_count()};
}
bool Type13NodeContributions::Matches(const type13::Model& model,const NodalNodeDomain& domain) const noexcept {
  return impl_&&impl_->model.Matches(model)&&impl_->domain.Matches(domain);
}
bool Type13NodeContributions::Matches(const Type13NodeContributions& other) const noexcept {
  return impl_&&other.impl_&&(impl_==other.impl_||Matches(other.impl_->model,other.impl_->domain));
}
std::size_t Type13NodeContributions::owned_payload_bytes() const noexcept {
  return impl_?impl_->bytes:sizeof(Type13NodeContributions);
}
std::size_t Type13NodeContributions::startup_payload_bytes() const noexcept {return owned_payload_bytes();}
} // namespace tl::fea
