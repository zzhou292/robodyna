// SPDX-License-Identifier: AGPL-3.0-or-later
#include "Beam18NodeContributions.h"
#include "../../lib_utils/BoundedArena.h"
#include <new>
namespace tl::fea {
struct Beam18NodeContributions::Impl {
  explicit Impl(const beam18::Model& value):model(value) {}
  beam18::Model model;
  util::HostArena arena;
  Beam18NodeContribution* records=nullptr;
  std::size_t count=0,bytes=0;
};
NodalDomainReport Beam18NodeContributions::Initialize(const beam18::Model& model,
    Beam18ContributionLimits limits) noexcept try {
  using S=NodalDomainStatus;
  if(impl_) return {S::AlreadyInitialized,"Beam18 contributions are immutable"};
  if(!model.prepared()||!model.domain()) return {S::InvalidInput,"Prepared beam model required"};
  const Beam18ContributionLimits hard;
  if(!limits.max_parents||limits.max_parents>hard.max_parents||!limits.max_nodes||
      limits.max_nodes>hard.max_nodes||!limits.max_host_bytes||limits.max_host_bytes>hard.max_host_bytes||
      model.parents().size()>limits.max_parents||model.domain()->node_count()>limits.max_nodes)
    return {S::ResourceLimit,"Beam18 contribution counts or caps exceed scope"};
  const auto bytes=model.owned_payload_bytes();
  const auto count=2*model.parents().size();
  util::BoundedArenaLayout arena(limits.max_host_bytes),budget(limits.max_host_bytes);
  util::ArenaRegion records,ignored;
  if(bytes<sizeof(beam18::Model)||!arena.Append<Beam18NodeContribution>(count,records)||
      !budget.Append<unsigned char>(sizeof(Beam18NodeContributions)+sizeof(Impl)+64,ignored)||
      !budget.Append<unsigned char>(arena.bytes(),ignored)||
      !budget.Append<unsigned char>(bytes-sizeof(beam18::Model),ignored))
    return {S::ResourceLimit,"Beam18 records and complete retained model exceed cap"};
  auto next=std::make_shared<Impl>(model);
  if(!next->arena.Initialize(arena.bytes())||!(next->records=next->arena.Construct<Beam18NodeContribution>(records)))
    return {S::ResourceLimit,"Beam18 contribution arena allocation failed"};
  for(std::size_t p=0;p<model.parents().size();++p) for(unsigned local=0;local<2;++local) {
    beam18::EndpointContribution value;
    if(!model.Endpoint(p,local,value)) return {S::InvalidInput,"Prepared beam endpoint query failed",p};
    next->records[2*p+local]={p,local,value};
  }
  next->count=count; next->bytes=budget.bytes(); impl_=std::move(next); return {};
} catch(const std::bad_alloc&) {return {NodalDomainStatus::ResourceLimit,"Beam18 contribution allocation failed"};}
const beam18::Model* Beam18NodeContributions::model() const noexcept {return impl_?&impl_->model:nullptr;}
const NodalNodeDomain* Beam18NodeContributions::domain() const noexcept {return impl_?impl_->model.domain():nullptr;}
std::size_t Beam18NodeContributions::record_count() const noexcept {return impl_?impl_->count:0;}
util::ConstView<Beam18NodeContribution> Beam18NodeContributions::records() const noexcept {
  static const Beam18NodeContribution empty; return {impl_?impl_->records:&empty,record_count()};
}
bool Beam18NodeContributions::Matches(const beam18::Model& model,const NodalNodeDomain& domain) const noexcept {
  return impl_&&impl_->model.Matches(model)&&impl_->model.domain()->Matches(domain);
}
bool Beam18NodeContributions::Matches(const Beam18NodeContributions& other) const noexcept {
  return impl_&&other.impl_&&(impl_==other.impl_||Matches(other.impl_->model,*other.domain()));
}
std::size_t Beam18NodeContributions::owned_payload_bytes() const noexcept {return impl_?impl_->bytes:sizeof(*this);}
std::size_t Beam18NodeContributions::startup_payload_bytes() const noexcept {return owned_payload_bytes();}
} // namespace tl::fea
