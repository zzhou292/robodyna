// SPDX-License-Identifier: AGPL-3.0-or-later
#include "ElementMassContributions.h"
#include "NodalDomainIdentity.h"
#include "../../lib_utils/BoundedArena.h"
#include "../../lib_utils/SourceIdentityIndex.h"
#include <cmath>
#include <new>
#include <stdexcept>

namespace tl::fea {
struct ElementMassContributions::Impl {
  explicit Impl(const NodalNodeDomain& d):domain(d) {}
  NodalNodeDomain domain;
  util::HostArena arena;
  ElementMassContribution* rows=nullptr;
  std::size_t count=0,retained=0,startup=0;
  double mass_scale=0;
};
namespace {
bool Same(double a,double b) noexcept {return std::memcmp(&a,&b,sizeof(double))==0;}
}
NodalDomainReport ElementMassContributions::Initialize(const NodalNodeDomain& domain,
    ElementMassInput input,ElementMassLimits limits) noexcept try {
  using S=NodalDomainStatus;
  if(impl_) return {S::AlreadyInitialized,"Element mass contributions are immutable"};
  if(!domain.prepared()) return {S::InvalidInput,"Prepared source node domain is required"};
  const ElementMassLimits hard;
  if(!limits.max_records||limits.max_records>hard.max_records||
      !limits.max_nodes||limits.max_nodes>hard.max_nodes||
      !limits.max_host_bytes||limits.max_host_bytes>hard.max_host_bytes||
      input.record_count>limits.max_records||domain.node_count()>limits.max_nodes)
    return {S::ResourceLimit,"Element mass counts or caps exceed scope"};
  if(!input.record_count||!std::isfinite(input.mass_to_kg)||input.mass_to_kg<=0||
      input.source_instance_id!=domain.source_instance_id())
    return {S::InvalidInput,"Element mass source instance, count or units differ"};

  util::BoundedArenaLayout arena(limits.max_host_bytes),retained(limits.max_host_bytes),
      startup(limits.max_host_bytes);
  util::ArenaRegion rows,ignored;
  const auto domain_bytes=domain.owned_payload_bytes();
  if(domain_bytes<sizeof(NodalNodeDomain)||
      !arena.Append<ElementMassContribution>(input.record_count,rows)||
      !retained.Append<unsigned char>(sizeof(*this)+sizeof(Impl)+64,ignored)||
      !retained.Append<unsigned char>(arena.bytes(),ignored)||
      !retained.Append<unsigned char>(domain_bytes-sizeof(NodalNodeDomain),ignored)||
      !startup.Append<unsigned char>(retained.bytes(),ignored)||
      !startup.Append<unsigned char>(util::SourceIdentityIndex<0>::Bytes(input.record_count),ignored))
    return {S::ResourceLimit,"Element mass records, domain and identity scratch exceed cap"};
  if(!nodal_domain_detail::ValidRange(input.records,input.record_count))
    return {S::InvalidInput,"Element mass input record range is invalid"};

  util::SourceIdentityIndex<0> identities;
  identities.Prepare(input.record_count,[&](std::size_t i) {return input.records[i].source_element_id;});
  auto next=std::make_shared<Impl>(domain);
  if(!next->arena.Initialize(arena.bytes())||
      !(next->rows=next->arena.Construct<ElementMassContribution>(rows)))
    return {S::ResourceLimit,"Element mass arena allocation failed"};
  for(std::size_t i=0;i<input.record_count;++i) {
    const auto& source=input.records[i];
    if(!source.source_element_id||!source.source_node_id||source.domain_node>=domain.node_count())
      return {S::InvalidInput,"Element mass EID, NID or domain index is invalid",i};
    if(identities.First(source.source_element_id)!=i)
      return {S::DuplicateIdentity,"Repeated original element-mass EID",i};
    if(domain.nodes()[source.domain_node].source_id!=source.source_node_id)
      return {S::MissingSource,"Element mass NID and domain index differ",i};
    if(!std::isfinite(source.mass_source)||source.mass_source<0)
      return {S::InvalidInput,"Element mass requires a finite nonnegative source value",i};
    const double mass=source.mass_source*input.mass_to_kg;
    if(!std::isfinite(mass)||(source.mass_source>0&&mass<=0))
      return {S::InvalidInput,"Element mass SI conversion overflow or positive underflow",i};
    next->rows[i]={source,mass};
  }
  next->count=input.record_count;
  next->mass_scale=input.mass_to_kg;
  next->retained=retained.bytes();
  next->startup=startup.bytes();
  impl_=std::move(next);
  return {};
} catch(const std::bad_alloc&) {
  return {NodalDomainStatus::ResourceLimit,"Element mass allocation failed"};
} catch(const std::length_error&) {
  return {NodalDomainStatus::ResourceLimit,"Element mass allocation size overflow"};
}
const NodalNodeDomain* ElementMassContributions::domain() const noexcept {
  return impl_?&impl_->domain:nullptr;
}
double ElementMassContributions::mass_to_kg() const noexcept {return impl_?impl_->mass_scale:0;}
tl::util::ConstView<ElementMassContribution> ElementMassContributions::records() const noexcept {
  static const ElementMassContribution empty;
  return {impl_?impl_->rows:&empty,impl_?impl_->count:0};
}
bool ElementMassContributions::Matches(const ElementMassContributions& other) const noexcept {
  if(!impl_||!other.impl_) return false;
  if(impl_==other.impl_) return true;
  if(!impl_->domain.Matches(other.impl_->domain)||impl_->count!=other.impl_->count||
      !Same(impl_->mass_scale,other.impl_->mass_scale)) return false;
  for(std::size_t i=0;i<impl_->count;++i) {
    const auto& a=impl_->rows[i].source;
    const auto& b=other.impl_->rows[i].source;
    if(a.source_element_id!=b.source_element_id||a.source_node_id!=b.source_node_id||
        a.domain_node!=b.domain_node||!Same(a.mass_source,b.mass_source)) return false;
  }
  return true;
}
std::size_t ElementMassContributions::owned_payload_bytes() const noexcept {
  return impl_?impl_->retained:sizeof(*this);
}
std::size_t ElementMassContributions::startup_payload_bytes() const noexcept {
  return impl_?impl_->startup:sizeof(*this);
}
} // namespace tl::fea
